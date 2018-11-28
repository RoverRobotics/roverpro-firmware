import configparser
from dataclasses import dataclass
from fnmatch import fnmatch
import os
import re
import subprocess
from pathlib import Path

# build script for C30 projects
import winreg
from winreg import HKEY_LOCAL_MACHINE, OpenKey, HKEY_CURRENT_USER


@dataclass
class BuildToolsC30:
    cc: Path
    hx: Path

    @classmethod
    def from_path(cls, path):
        return cls(
            cc=Path(path, 'bin', 'pic30-gcc.exe'),
            hx=Path(path, 'bin', 'pic30-bin2hex.exe'),
        )

    @classmethod
    def from_registry(cls, guid_str):
        if guid_str == '{479DDE59-4D56-455E-855E-FFF59A3DB57E}':
            with OpenKey(HKEY_CURRENT_USER, r"Software\Microchip\MPLAB IDE\Tool Locations") as key:
                cc, _ = winreg.QueryValueEx(key, 'T_dsPICcc.C30')
                hx, _ = winreg.QueryValueEx(key, 'T_dsPICbin2hex.C30')
            return cls(cc=Path(cc), hx=Path(hx))
        elif guid_str == '{9BCCB495-CD65-480A-BA76-63D8E78B117F}':
            raise ValueError('XC16 not yet supported')
        else:
            raise NotImplementedError


class MPLabProject:
    def __init__(self, path, debug_build=False):
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

    def get_include_flags(self):
        inc_flags = []
        for a_dir in self.config['PATH_INFO']['dir_inc'].split(';'):
            inc_flags.append('-I')
            inc_flags.append(a_dir)
        return inc_flags

    def get_build_tools(self):
        return BuildToolsC30.from_registry(self.config['SUITE_INFO']['SUITE_GUID'])

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

        # gcc-c30 tool flags
        file_build_flags = self.get_tool_flags('{25AC22BD-2378-4FDB-BFB6-7345A15512D3}', file_id)
        return [str(self.get_build_tools().cc),
                '-mcpu=' + m[1],
                '-c', str(self.get_file_source_path(file_id)),
                '-o', str(self.get_file_object_path(file_id)),
                *self.get_include_flags(),
                *(['-D__DEBUG'] if self.debug_build else [])
                *file_build_flags.split(' '),
                ]

    def get_build_args_for_project(self):
        obj_files = [str(self.get_file_object_path(fid)) for fid in self.get_source_file_ids()]
        device = self.config['HEADER']['device']
        m = re.match('PIC(.+)', device)
        device_gld = '-Tp' + m[1] + '.gld'

        args = [
            str(self.get_build_tools().cc),
            '-mcpu=' + m[1],
            *obj_files,
            '-o' + self.config['PATH_INFO']['dir_bin'] + "/" + self.project_name + '.cof',
            ("-Wl,-L" + self.config['PATH_INFO']['dir_lib'] + "," + device_gld + ",--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1").split(' ')
        ]
        return args

    def get_build_args_for_distribution(self):
        hx = self.get_build_tools().hx
        return [str(hx), self.config['PATH_INFO']['dir_bin'] + "\\" + self.project_name + '.cof']


def main():
    mcpfiles = [f.absolute() for f in Path('.').rglob('*.mcp')]
    for mcpfile in mcpfiles:
        cwd = os.getcwd()
        try:
            # p = MPLabProject(r"C:\Users\dan\Documents\OpenRoverFirmware-dan\Power_Board\PowerBoard.mcp")
            p = MPLabProject(mcpfile)
            print('Switching to directory: ' + str(p.path.parent))
            os.chdir(p.path.parent)
            p.ensure_output_dirs()

            for f in p.get_source_file_ids():
                args = p.get_build_args_for_file(f)
                print('Executing: ' + subprocess.list2cmdline(args))
                subprocess.check_call(args)

            args = p.get_build_args_for_project()
            subprocess.check_call(args)

            args = p.get_build_args_for_distribution()
            subprocess.check_call(args)
        finally:
            os.chdir(cwd)

    # Path('./bin/' + p.project_name + '.hex').replace('PowerBoard-' + subprocess.check_output(['git', 'describe', '--tags']).decode().strip() + "-release.hex")


if __name__ == '__main__':
    main()
