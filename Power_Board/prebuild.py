#! python3.7

import datetime
import re
import subprocess
import sys

targetpath = sys.argv[1]
now = datetime.datetime.now()
print('Writing generated file to ' + targetpath)


def syscall(*args: str):
    print('calling ' + subprocess.list2cmdline(args))
    result = subprocess.check_output(args, text=True).strip()
    print('returned ' + result)
    return result


git_commit_id = syscall('git', 'rev-parse', '--short', 'HEAD')
assert re.fullmatch(r'[\w]+', git_commit_id)

git_branch = syscall('git', 'rev-parse', '--abbrev-ref', 'HEAD')
assert re.fullmatch(r'[\w/.\-]+', git_branch)

git_branch_match = re.fullmatch(r'release/(.*)', git_branch)
if git_branch_match:
    print('Basing version number on branch:', git_branch)
    [baseversion] = git_branch_match.groups()
    git_commits_since_branch = syscall('git', 'rev-list', '--count', '^' + git_branch, 'HEAD')
    assert re.fullmatch(r'\d+', git_commits_since_branch)
    label = 'rc' + git_commits_since_branch + '+' + git_commit_id
else:
    baseversion = git_tag = syscall('git', 'describe', '--abbrev=0')
    print('Basing version number on tag:', git_tag)
    git_commits_since_tag = syscall('git', 'rev-list', '--count', '^' + git_tag, 'HEAD')
    assert re.fullmatch(r'\d+', git_commits_since_tag)
    if int(git_commits_since_tag) == 0:
        label = ''
    else:
        label = 'post' + git_commits_since_tag + '+' + git_commit_id

major, minor, patch = re.fullmatch(r'(\d+)\.(\d+)\.(\d+)', baseversion).groups()
version = '{}.{}.{}{}'.format(major, minor, patch, label)

print('version determined to be:', version)
with open(targetpath, 'w') as f:
    f.write(f"""\
/// @file
/// Firmware version information.
/// Generated at time of build by {__file__}
#ifndef VERSION_GENERATED_H
#define VERSION_GENERATED_H

#include <stdint.h>

#define RELEASE_VERSION_NAME "{version}"
/// Semantic version as per semver.org
typedef struct {{
	/// serial number for incompatible API changes
    unsigned major;
    /// serial number for feature releases
    unsigned minor;
    /// serial number for bug fixes
    unsigned patch;
    /// string value to track ad-hoc changes and non-released versions
    char prerelease[];
}} ReleaseVersion;
#define RELEASE_VERSION {{{major}, {minor}, {patch}, "{label}"}}
#define RELEASE_VERSION_FLAT {int(major) * 10000 + int(minor) * 100 + int(patch)}
#define BUILD_DATE "{now.date().isoformat()}"
#define BUILD_TIME "{now.time().replace(microsecond=0).isoformat()}"
#endif
""")
