import configparser
from fnmatch import fnmatch
import subprocess
from pathlib import Path

# build script for C30 projects
import winreg
from winreg import HKEY_LOCAL_MACHINE, OpenKey, HKEY_CURRENT_USER

project = 'firmware.mcp'
target_base = Path(project).stem

config = configparser.ConfigParser()
config.read(project)

global_file_build_flags = config['TOOL_SETTINGS']['TS{25AC22BD-2378-4FDB-BFB6-7345A15512D3}']
project_build_flags = config['TOOL_SETTINGS']['TS{7DAC9A1D-4C45-45D6-B25A-D117C74E8F5A}']

device = config['HEADER']['device']
dir_bin = config['PATH_INFO']['dir_bin']
dir_tmp = config['PATH_INFO']['dir_tmp']
dir_inc = config['PATH_INFO']['dir_inc']
dir_lib = config['PATH_INFO']['dir_lib']
filter_src = config['CAT_FILTERS']['filter_src']

suite_guid = config['SUITE_INFO']['SUITE_GUID']

inc_flags = []
for f in dir_inc.split(';'):
    inc_flags.append('-I')
    inc_flags.append(f)
assert device[:3] == 'PIC'

assert suite_guid == '{479DDE59-4D56-455E-855E-FFF59A3DB57E}'  # MPLAB C30

# Computer\HKEY_CURRENT_USER\Software\Microchip\MPLAB IDE\Tool Locations
with OpenKey(HKEY_CURRENT_USER, r"Software\Microchip\MPLAB IDE\Tool Locations") as key:
    # keyT_dsPICcc.C30
    s = winreg.QueryInfoKey(key)
    gcc, _ = winreg.QueryValueEx(key, 'T_dsPICcc.C30')
    bin2hex, _ = winreg.QueryValueEx(key, 'T_dsPICbin2hex.C30')


def expand_mplab_macros(s):
    MPLAB_MACROS = {
        'BINDIR_':      dir_bin + "/",
        'TARGETBASE':   target_base,
        'TARGETSUFFIX': 'cof',
    }
    result = s
    for k, v in MPLAB_MACROS.items():
        result = result.replace('$(' + k + ')', v)
    return result


gcc_cpu_flag = '-mcpu=' + device[3:]
device_gld = '-Tp' + device[3:] + '.gld'


def is_src_file(name):
    return any(fnmatch(name, pat) for pat in filter_src.split(';'))


def compile_file(key, filename):
    assert key[:5] == 'file_'
    file_build_flags = None
    if config['TOOL_SETTINGS'].get('TS{25AC22BD-2378-4FDB-BFB6-7345A15512D3}' + key[5:] + '_active') == 'yes':
        file_build_flags = config['TOOL_SETTINGS'].get('TS{25AC22BD-2378-4FDB-BFB6-7345A15512D3}' + key[5:])
    if file_build_flags is None:
        file_build_flags = global_file_build_flags

    path = Path(filename)
    if path.suffix == '.c' or path.suffix == '.s':
        obj_file = dir_tmp + '/' + path.stem + '.o'

        args = [
            gcc,
            gcc_cpu_flag,
            '-c', str(path),
            '-o', obj_file,
            *inc_flags,
            *expand_mplab_macros(file_build_flags).split(' ')
        ]
        print('Executing: ' + subprocess.list2cmdline(args))
        subprocess.check_call(args)
        return obj_file


def main():
    obj_files = [compile_file(k, v) for k, v in config['FILE_INFO'].items() if is_src_file(v)]

    args = [
        gcc,
        gcc_cpu_flag,
        *obj_files,
        '-o' + dir_bin + "/" + target_base + '.cof',
        ("-Wl,-L" + dir_lib + "," + device_gld + ",--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1").split(' ')
    ]

    print('Executing: ' + subprocess.list2cmdline(args))
    subprocess.check_call(args)

    subprocess.check_call([bin2hex, dir_bin + "\\" + target_base + '.cof'])

    Path('./bin/' + target_base + '.hex').replace('PowerBoard-' + subprocess.check_output(['git', 'describe', '--tags']).decode().strip() + "-release.hex")


if __name__ == '__main__':
    main()
