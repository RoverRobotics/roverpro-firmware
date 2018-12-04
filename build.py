#! python3.7


import configparser
from dataclasses import dataclass
from fnmatch import fnmatch
import logging
import os
from pathlib import Path
import re
import subprocess
import winreg
from winreg import HKEY_CURRENT_USER, OpenKey


def log_and_run(args):
    logging.debug('Executing: %s', subprocess.list2cmdline(args))
    result = subprocess.run(args, capture_output=True)
    if result.stdout:
        logging.info(result.stdout.decode())
    if result.stderr:
        logging.warning(result.stderr.decode())

    result.check_returncode()


class BuildToolSuite:
    cc: Path
    cc_uuid: str
    hx: Path


@dataclass
class BuildToolsXC16(BuildToolSuite):
    cc: Path
    cc_uuid = '{F9CE474D-6A6C-401D-A11E-BEE01B244D79'
    hx: Path

    @classmethod
    def from_path(cls, path):
        return cls(
            cc=Path(path, 'bin', 'xc16-gcc.exe'),
            hx=Path(path, 'bin', 'xc16-bin2hex.exe'),
        )

    @classmethod
    def from_registry(cls):
        with OpenKey(HKEY_CURRENT_USER, r"Software\Microchip\MPLAB IDE\Tool Locations") as key:
            cc, _ = winreg.QueryValueEx(key, 'T_XC16cc.XC16')
            hx = Path(cc).parent / 'xc16-bin2hex.exe'
        return cls(cc=Path(cc), hx=Path(hx))


@dataclass
class BuildToolsC30(BuildToolSuite):
    cc: Path
    cc_uuid = '{25AC22BD-2378-4FDB-BFB6-7345A15512D3}'
    hx: Path

    @classmethod
    def from_path(cls, path):
        return cls(
            cc=Path(path, 'bin', 'pic30-gcc.exe'),
            hx=Path(path, 'bin', 'pic30-bin2hex.exe'),
        )

    @classmethod
    def from_registry(cls):
        with OpenKey(HKEY_CURRENT_USER, r"Software\Microchip\MPLAB IDE\Tool Locations") as key:
            cc, _ = winreg.QueryValueEx(key, 'T_dsPICcc.C30')
            hx, _ = winreg.QueryValueEx(key, 'T_dsPICbin2hex.C30')
        return cls(cc=Path(cc), hx=Path(hx))


class MPLabProject:
    def __init__(self, path, debug_build=False, optimization=None):
        self.path = Path(path).absolute()
        self.config = configparser.ConfigParser()
        self.config.read(self.path)
        self.debug_build = debug_build

    def ensure_output_dirs(self):
        for dir_id in ['dir_tmp', 'dir_bin']:
            dir_path = self.config['PATH_INFO'][dir_id]
            if dir_path != '':
                for path in dir_path.split(';'):
                    os.makedirs(path, exist_ok=True)

    @property
    def tool_suite(self):
        u = self.config['SUITE_INFO']['SUITE_GUID'].casefold()

        if u == '{479DDE59-4D56-455E-855E-FFF59A3DB57E}'.casefold():
            return BuildToolsC30.from_registry()
        elif u == '{9BCCB495-CD65-480A-BA76-63D8E78B117F}'.casefold():
            return BuildToolsXC16.from_registry()
        else:
            raise NotImplementedError('unknown toolsuite %s', u)

    @property
    def project_name(self):
        return self.path.stem

    def is_source_file(self, filename):
        filter_src = self.config['CAT_FILTERS']['filter_src']
        return any(fnmatch(filename, pat) for pat in filter_src.split(';'))

    def get_source_file_ids(self):
        for file_id, filename in self.config['FILE_INFO'].items():
            if self.is_source_file(filename):
                m = re.match('file_(.+)', file_id)
                yield m[1]

    def get_file_source_path(self, file_id):
        return Path(self.config['PATH_INFO']['dir_src'], self.config['FILE_INFO']['file_' + file_id])

    def get_file_object_path(self, file_id):
        filename = Path(self.config['FILE_INFO']['file_' + file_id]).with_suffix('.o').name
        return Path(self.config['PATH_INFO']['dir_tmp'], filename)

    @property
    def built_artifact(self):
        return os.path.join(self.config['PATH_INFO']['dir_bin'], self.project_name + '.hex')

    def get_include_flags(self):
        inc_flags = []
        if self.config['PATH_INFO']['dir_inc'] != '':
            for a_dir in self.config['PATH_INFO']['dir_inc'].split(';'):
                inc_flags.append('-I')
                inc_flags.append(a_dir)
        return inc_flags

    def expand_mplab_macros(self, s):
        MPLAB_MACROS = {
            'BINDIR_':      str(self.config['PATH_INFO']['dir_bin']) + '/',
            'TARGETBASE':   self.path.stem,
            'TARGETSUFFIX': 'cof',
        }
        result = s
        for k, v in MPLAB_MACROS.items():
            result = result.replace('$(' + k + ')', v)
        return result

    def get_tool_flags(self, tool_guid, file_id):
        assert re.match('{.+}', tool_guid)
        result = None
        if self.config['TOOL_SETTINGS'].get('TS' + tool_guid + file_id + '_active') == 'yes':
            result = self.config['TOOL_SETTINGS'].get('TS' + tool_guid + file_id)
        if result is None:
            result = self.config['TOOL_SETTINGS'].get('TS' + tool_guid)

        assert result is not None
        return self.expand_mplab_macros(result)

    def get_build_args_for_file(self, file_id):
        device = self.config['HEADER']['device']
        m = re.match('PIC(.+)', device)

        file_build_flags = self.get_tool_flags(self.tool_suite.cc_uuid, file_id)
        return [str(self.tool_suite.cc),
                '-mcpu=' + m[1],
                '-c', str(self.get_file_source_path(file_id)),
                '-o', str(self.get_file_object_path(file_id)),
                *self.get_include_flags(),
                *(['-D__DEBUG'] if self.debug_build else []),
                *file_build_flags.split(' '),
                ]

    def get_build_args_for_project(self):
        obj_files = [str(self.get_file_object_path(fid)) for fid in self.get_source_file_ids()]
        device = self.config['HEADER']['device']
        m = re.match('PIC(.+)', device)
        device_gld = '-Tp' + m[1] + '.gld'

        linker_options = '-Wl,' + ','.join([
            *(["-L" + self.config['PATH_INFO']['dir_lib']] if self.config['PATH_INFO']['dir_lib'] else []),
            device_gld,
            '--defsym=__MPLAB_DEBUG=1"'
        ])

        args = [
            str(self.tool_suite.cc),
            '-mcpu=' + m[1],
            *obj_files,
            '-o', os.path.join(self.config['PATH_INFO']['dir_bin'], self.project_name + '.cof'),
            linker_options.split(' ')
        ]
        return args

    def get_build_args_for_distribution(self):
        hx = self.tool_suite.hx
        return [str(hx), os.path.join(self.config['PATH_INFO']['dir_bin'], self.project_name + '.cof')]


def main():
    try:
        import coloredlogs
        coloredlogs.install(level='DEBUG')
    except ModuleNotFoundError:
        logging.warning('Could not colorize logs. pip install coloredlogs to set this up.')

    # base_dir = r'C:\Users\dan\Documents\RobotexFirmware\Avatar_Micro_Robot\Robot_V1_Accessories\PTZ_Camera_V1\trunk'
    base_dir = os.path.dirname(os.path.realpath(__file__))
    logging.info('Building all project in directory: %s', base_dir)
    mcp_files = [f.absolute() for f in Path(base_dir).rglob('*.mcp')]
    for mcp_file in mcp_files:
        cwd = os.getcwd()
        try:
            p = MPLabProject(mcp_file)
            logging.info('Beginning build of %s', mcp_file)
            project_dir = p.path.parent
            logging.debug('Switching to directory: %s', project_dir)
            os.chdir(project_dir)
            p.ensure_output_dirs()

            for f in p.get_source_file_ids():
                logging.info('Building file: %s', p.get_file_source_path(f).name)
                args = p.get_build_args_for_file(f)
                log_and_run(args)

            logging.info('Linking project: %s', mcp_file)
            args = p.get_build_args_for_project()
            log_and_run(args)

            logging.info('Packaging project binaries')
            args = p.get_build_args_for_distribution()
            log_and_run(args)

            binary_path = os.path.join(project_dir, p.built_artifact)
            assert os.path.isfile(binary_path)
            logging.info('Build of project %s succeeded!\nResulting binary at:%s', mcp_file, binary_path)
        except subprocess.CalledProcessError as e:
            logging.exception('Build of project %s failed in subprocess:\n%s\nreturn code %s\nstderr:\n%s\nstdout:\n%s', mcp_file, subprocess.list2cmdline(e.cmd), e.returncode, e.stderr.decode(),
                              e.stdout.decode())
        except Exception as e:
            logging.exception('Build of project %s failed with error:\n%s', mcp_file, e)
        finally:
            os.chdir(cwd)

if __name__ == '__main__':
    main()
