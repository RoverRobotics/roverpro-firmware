import asyncio
import configparser
from dataclasses import dataclass
from fnmatch import fnmatch
import logging
import os
from pathlib import Path
import re
from subprocess import list2cmdline
import winreg


class BuildToolSuite:
    cc: Path
    cc_uuid: str
    hx: Path


@dataclass
class BuildToolsXC16(BuildToolSuite):
    cc: Path
    cc_uuid = '{F9CE474D-6A6C-401D-A11E-BEE01B244D79}'
    hx: Path

    @classmethod
    def from_path(cls, path):
        return cls(
            cc=Path(path, 'bin', 'xc16-gcc.exe'),
            hx=Path(path, 'bin', 'xc16-bin2hex.exe'),
        )

    @classmethod
    def from_registry(cls):
        with winreg.OpenKey(winreg.HKEY_CURRENT_USER, r"Software\Microchip\MPLAB IDE\Tool Locations") as key:
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
        with winreg.OpenKey(winreg.HKEY_CURRENT_USER, r"Software\Microchip\MPLAB IDE\Tool Locations") as key:
            cc, _ = winreg.QueryValueEx(key, 'T_dsPICcc.C30')
            hx, _ = winreg.QueryValueEx(key, 'T_dsPICbin2hex.C30')
        return cls(cc=Path(cc), hx=Path(hx))


class MPLabProject:
    def __init__(self, path, debug_build=False, optimization=None):
        self.path = Path(path).absolute()
        self.config = configparser.ConfigParser()
        self.config.read(self.path)
        self.debug_build = debug_build

    @property
    def base_dir(self):
        return self.path.parent

    def __str__(self):
        return f'{type(self).__name__}({self.path})'

    async def exec_subprocess(self, args):
        if isinstance(args, str):
            argstring = args
        else:
            argstring = list2cmdline(args)
        logging.debug('Executing: %s', argstring)
        process = await asyncio.create_subprocess_exec(*args, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE, cwd=self.path.parent)
        await process.wait()
        stdout = (await process.stdout.read(-1)).decode()
        if stdout:
            logging.info('stdout=\n' + stdout)
        stderr = (await process.stderr.read(-1)).decode()
        if stderr:
            logging.warning('stderr=\n' + stderr)
        assert process.returncode == 0

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

    def is_linker_file(self, filename):
        filter_lkr = self.config['CAT_FILTERS']['filter_lkr'] or '*.gld'
        return any(fnmatch(filename, pat) for pat in filter_lkr.split(';'))

    def get_source_file_ids(self):
        result = []
        for file_id, filename in self.config['FILE_INFO'].items():
            if self.is_source_file(filename):
                m = re.match('file_(.+)', file_id)
                result.append(m.group(1))
        return result

    def get_file_source_path(self, file_id):
        return Path(self.base_dir, self.config['PATH_INFO']['dir_src'], self.config['FILE_INFO']['file_' + file_id])

    def get_file_object_path(self, file_id):
        filename = Path(self.config['FILE_INFO']['file_' + file_id]).with_suffix('.o').name
        return Path(self.base_dir, self.config['PATH_INFO']['dir_tmp'], filename)

    @property
    def built_artifact(self):
        return os.path.join(self.base_dir, self.config['PATH_INFO']['dir_bin'], self.project_name + '.hex')

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

    async def file_build(self, file_id):
        obj_file = self.get_file_object_path(file_id)
        try:
            os.unlink(obj_file)
        except FileNotFoundError:
            pass

        file_build_flags = self.get_tool_flags(self.tool_suite.cc_uuid, file_id)
        e = await self.exec_subprocess([str(self.tool_suite.cc),
                                        self.cpu_flag,
                                        '-c', str(self.get_file_source_path(file_id)),
                                        '-o', str(self.get_file_object_path(file_id)),
                                        *self.get_include_flags(),
                                        *(['-D__DEBUG'] if self.debug_build else []),
                                        *file_build_flags.split(' ')])

        assert obj_file.exists()
        return obj_file

    @property
    def cpu_flag(self):
        device = self.config['HEADER']['device']
        m = re.match('PIC(.+)', device)
        return '-mcpu=' + m[1]

    @property
    def linker_script(self):
        scripts = [fn for fn in self.config['FILE_INFO'].values() if self.is_linker_file(fn)]

        if len(scripts) > 1:
            logging.error('Multiple linker scripts found - only the first one will be used')

        if len(scripts) == 0:
            device = self.config['HEADER']['device']
            m = re.match('PIC(.+)', device)
            assert m
            return 'p' + m[1] + '.gld'

        return scripts[0]

    async def link_project(self, obj_files=None, linker_script=None):
        cof_file = os.path.join(self.base_dir, self.config['PATH_INFO']['dir_bin'], self.project_name + '.cof')
        try:
            os.unlink(cof_file)
        except FileNotFoundError:
            pass
        if obj_files is None:
            obj_files = [str(self.get_file_object_path(fid)) for fid in self.get_source_file_ids()]

        linker_options = '-Wl,' + ','.join([
            *(["-L" + self.config['PATH_INFO']['dir_lib']] if self.config['PATH_INFO']['dir_lib'] else []),
            '-T' + self.linker_script,
            '--defsym=__MPLAB_DEBUG=1'
        ])

        args = [
            str(self.tool_suite.cc),
            self.cpu_flag,
            *obj_files,
            '-o', cof_file,
            linker_options
        ]
        await self.exec_subprocess(args)
        assert os.path.isfile(cof_file)
        return cof_file

    async def hexify(self):
        hex_file = os.path.join(self.base_dir, self.config['PATH_INFO']['dir_bin'], self.project_name + '.hex')
        try:
            self.unlink = os.unlink(hex_file)
        except FileNotFoundError:
            pass
        assert not os.path.isfile(hex_file)
        hx = self.tool_suite.hx
        await self.exec_subprocess([str(hx), os.path.join(self.config['PATH_INFO']['dir_bin'], self.project_name + '.cof')])
        assert os.path.isfile(hex_file)
        return hex_file

    async def project_prebuild(self):
        if self.config.getboolean('CUSTOM_BUILD', 'Pre-BuildEnabled'):
            value = self.config.get('CUSTOM_BUILD', 'Pre-Build')
            if value:
                return await self.exec_subprocess(value.split(' '))

        logging.debug('No prebuild step')
        return

    async def project_postbuild(self):
        if self.config.getboolean('CUSTOM_BUILD', 'Post-BuildEnabled'):
            value = self.config.get('CUSTOM_BUILD', 'Post-Build')
            if value:
                return await self.exec_subprocess(value.split(' '))

        logging.debug('No postbuild step')
        return
