#! python3.7

import subprocess
import datetime
import re
version = subprocess.check_output(['git', 'describe', '--tags']).decode().strip()
with open('include/version.GENERATED.h', 'w') as f:
    m = re.match(r'(\d+)\.(\d+)\.(\d+)[^\d](.*)', version)
    major, minor, patch, prerelease = m.groups()
    prerelease_c_str = '"' + prerelease.replace('\\', '_').replace('"', '_') + '"'

    f.write(f"""
// Generated file
// Created by {__file__} at {datetime.datetime.now()}
#ifndef VERSION_GENERATED_H
#define VERSION_GENERATED_H

#include <stdint.h>

const char RELEASE_VERSION_NAME[] = "{version}";
const struct {{
    unsigned major;
    unsigned minor;
    unsigned patch;
    char prerelease[];
}} RELEASE_VERSION={{{major}, {minor}, {patch}, {prerelease_c_str}}};
uint16_t RELEASE_VERSION_FLAT={int(major) * 10000 + int(minor) * 100 + int(patch)};
#endif
""")
