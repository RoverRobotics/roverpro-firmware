#! python3.7
import argparse
import asyncio
import configparser
import logging
import os
from pathlib import Path
import urllib.parse
import webbrowser

import coloredlogs
import github

from mplab import MPLabProject

parser = argparse.ArgumentParser(description='Build the OpenRover firmware')
parser.add_argument('--debug', action='store_true', help='Build in debug mode')
parser.add_argument('--upload', action='store_true', help='Create a release draft on GitHub when done building')
parser.add_argument('--verbose', '-v', action='count', help='Log more verbosely. --v = INFO, --vv = DEBUG')


async def build_project(p):
    logging.info('Beginning build of %s', p)

    p.ensure_output_dirs()

    logging.info('Executing pre-build step')
    await p.project_prebuild()

    source_file_ids = p.get_source_file_ids()
    binaries = await asyncio.gather(*[p.file_build(f) for f in source_file_ids])

    logging.info('Linking project')
    await p.link_project([str(b) for b in binaries])

    logging.info('Packaging project binaries')
    hex_file = await p.hexify()

    logging.info('Build of project succeeded!\nResulting binary at:%s', hex_file)

    logging.info('Executing post-build step')
    await p.project_postbuild()
    return hex_file


async def main():
    command_line_options = parser.parse_args()
    if command_line_options.verbose is None:
        log_level = 'WARNING'
    elif command_line_options.verbose == 1:
        log_level = 'INFO'
    else:
        log_level = 'DEBUG'
    coloredlogs.install(level=log_level)
    asyncio.set_event_loop(asyncio.ProactorEventLoop())
    base_dir = os.path.dirname(os.path.realpath(__file__))
    logging.info('Building all project in directory: %s', base_dir)
    mcp_files = [f.absolute() for f in Path(base_dir).rglob('*.mcp')]

    project_tasks = []
    for mcp_file in mcp_files:
        mplab_project = MPLabProject(mcp_file, debug_build=command_line_options.debug)
        project_tasks.append(build_project(mplab_project))

    hexfiles = await asyncio.gather(*project_tasks)

    if command_line_options.upload:
        logging.info('Uploading to git...')
        import git

        local = git.Repo(base_dir, search_parent_directories=True)
        SECTION = 'github-release'
        git_file = os.path.join(local.git_dir, 'config')
        with  local.config_writer() as git_config:
            try:
                git_config.add_section(SECTION)
            except configparser.DuplicateSectionError:
                pass

            def get_value(key):
                result = git_config.get_value(SECTION, key, '')
                if not result:
                    git_config.set_value(SECTION, key, '')
                    logging.error(f'Missing config for releases: {SECTION}.{key}')
                return result

            repo = get_value('repo')
            user = get_value('user')
            password = get_value('password')

        if not all([repo, user, password]):
            logging.fatal(f'Please add missing info to git config at {git_file}')
            exit(1)

        if local.is_dirty():
            logging.warning('Local branch is dirty!')

        commit = local.head.commit
        tag = None
        for t in local.tags:
            if t.commit == commit:
                tag = t
                break

        if tag is None:
            logging.warning('Current commit has no tag!')
        else:
            logging.info(f'Current commit is tagged {tag}')

        logging.info('Pushing local head and tag to git')
        local.git.push(f'https://{urllib.parse.quote(user)}:{urllib.parse.quote(password)}@github.com/{repo}', tag)

        prev_release_commits = set()
        for t in local.tags:
            if t != tag:
                prev_release_commits.update(t.commit.iter_parents())

        prev_messages = []
        for c in commit.iter_parents():
            if c not in prev_release_commits:
                prev_messages.append(c.message)

        logging.info(f'Logging into github as {user} for repo {repo}')
        g = github.Github(user, password)
        remote = g.get_repo(repo)

        logging.info(f'Uploading to GitHub...')
        release = remote.create_git_release(
            tag=tag.name if tag else '',
            name=tag.name if tag else commit.name_rev,
            message='\n'.join(prev_messages),
            draft=True,
            target_commitish=commit.hexsha
        )
        logging.info('Created GitHub release. Adding assets...')

        for b in hexfiles:
            p = Path(b)
            if tag:
                label = f'{p.stem}-{tag}{p.suffix}'
            else:
                label = f'{p.stem}{p.suffix}'
            release.upload_asset(b, label=label)
        logging.info('Done!!!')

        webbrowser.open(release.html_url)


if __name__ == '__main__':
    asyncio.ProactorEventLoop().run_until_complete(main())
