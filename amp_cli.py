import docker
import os
import subprocess

import click


@click.group()
def cli():
    """This is the official AMP Software Sub-Team CLI Tool that helps you 
    develop on AMP_ASSv2 using docker containers in your current environment.

    In order to run some of the functions, you must be in the root directory 
    of the AMP_ASSv2 repo. If there are any features you would like to see,
    or bugs that need to be fixed, feel free to open an issue or PR!
    """
    pass


@cli.command()
@click.option('-b', '--build', is_flag=True,
              help='Build the frame container, if not up-to-date.')
@click.option('--display', default='mesa',
              type=click.Choice(['mesa', 'nvidia'], case_sensitive=False),
              help='Choose display driver option.')
def devel(build: bool, display: str):
    """Develop with current directory mounted on the container.
    """
    # Initialize docker client environment
    try:
        client = docker.from_env()
    except Exception as exception:
        raise click.ClickException(
            f'Ran into an error, look at the help menu.\nError: {exception}')
    cwd = os.getcwd()
    tag = 'amp-devel:frame-desktop'

    # Make sure current directory is AMP_ASSv2
    if cwd.split('/')[-1] != 'AMP_ASSv2':
        raise click.ClickException(click.style(
            'Devel only works when executed from the root directory of the AMP_ASSv2 local repo.', fg='red'))

    if build:
        click.echo('Building containers...')

        click.echo('Step 1/2: Building amp-devel:noetic-desktop.')
        try:
            _ = client.images.build(
                path=cwd, tag='amp-devel:noetic-desktop',
                dockerfile=(cwd+'/docker/desktop.Dockerfile'))
            click.secho(
                ' ---> Finished building amp-devel:noetic-desktop.', fg='yellow')
        except Exception as exception:
            raise click.ClickException(
                f'Ran into an error, look at the help menu.\nError: {exception}')

        click.echo(f'Step 2/2: Building {tag}.')
        try:
            _ = client.images.build(
                path=cwd, tag=tag,
                dockerfile=(cwd+'/docker/frame.Dockerfile'))
            click.secho(
                f' ---> Finished building {tag}.', fg='yellow')
        except Exception as exception:
            raise click.ClickException(
                f'Ran into an error, look at the help menu.\nError: {exception}')

    try:
        # Run the contianer
        click.echo(f'Running {tag}...')
        if display == 'mesa':
            container = client.containers.run(
                tag, stdin_open=True, tty=True, auto_remove=True,
                network_mode='host',
                name='amp-assv2-scratch',
                environment={
                    'DISPLAY': os.getenv('DISPLAY'),
                    'QT_X11_NO_MITSHM': 1,
                },
                devices=['/dev/dri:/dev/dri'],
                volumes={
                    '/tmp/.X11-unix': {
                        'bind': '/tmp/.X11-unix',
                        'mode': 'rw',
                    },
                    cwd: {
                        'bind': '/amp_ws',
                        'mode': 'rw',
                    },
                },
                privileged=True,
                detach=True)
        # TODO: NVidia settings
        else:
            raise click.ClickException('NVidia config WIP')
        # Attach to the container
        click.echo('Use [<Ctrl> + D] to stop and exit the container.')
        click.secho(
            f'Entering container {container.name}:{container.short_id}.', fg='blue')
        subprocess.run(['bash', '-c', 'xhost +local:docker'])
        subprocess.run(['bash', '-c', f'docker attach {container.id}'])
        subprocess.run(['bash', '-c', 'xhost -local:docker'])
        click.secho('Successfully exited and stopped container.', fg='green')
    except docker.errors.ImageNotFound:
        raise click.ClickException(
            f'Unable to find {tag}, maybe try running with "--build" flag?')
    except Exception as exception:
        raise click.ClickException(
            f'Ran into an error, look at the help menu.\nError {exception}')


@cli.command()
@click.option('-b', '--build', is_flag=True,
              help='Build the corresponding containers.')
@click.option('--display', default='mesa', show_default=True,
              type=click.Choice(['mesa', 'nvidia'], case_sensitive=False),
              help='Choose display driver option.')
def scratch(build: bool, display: str):
    """Develop on a fully isolated container.
    """
    # Initialize docker client environment
    try:
        client = docker.from_env()
    except Exception as exception:
        raise click.ClickException(
            f'Ran into an error, look at the help menu.\nError: {exception}')
    cwd = os.getcwd()

    tag = f'amp-devel:{display}-build'
    dockerfile = cwd + f'/docker/{display}.Dockerfile'

    # Build container
    if build:
        # Make sure current directory is AMP_ASSv2
        if cwd.split('/')[-1] != 'AMP_ASSv2':
            raise click.ClickException(click.style(
                'Build only works inside root directory of the AMP_ASSv2 local repository.', fg='red'))

        click.echo('Building containers...')

        click.echo('Step 1/3: Building amp-devel:noetic-desktop.')
        try:
            _ = client.images.build(
                path=cwd, tag='amp-devel:noetic-desktop',
                dockerfile=(cwd+'/docker/desktop.Dockerfile'))
            click.secho(
                ' ---> Finished building amp-devel:noetic-desktop.', fg='yellow')
        except Exception as exception:
            raise click.ClickException(
                f'Ran into an error, look at the help menu.\nError {exception}')

        click.echo('Step 2/3: Building amp-devel:frame-desktop.')
        try:
            _ = client.images.build(
                path=cwd, tag='amp-devel:frame-desktop',
                dockerfile=(cwd+'/docker/frame.Dockerfile'))
            click.secho(
                ' ---> Finished building amp-devel:frame-desktop.', fg='yellow')
        except Exception as exception:
            raise click.ClickException(
                f'Ran into an error, look at the help menu.\nError {exception}')

        click.echo(f'Step 3/3: Building {tag}.')
        try:
            _ = client.images.build(
                path=cwd, tag=tag, dockerfile=dockerfile)
            click.secho(f' ---> Finished building {tag}.', fg='yellow')
        except Exception as exception:
            raise click.ClickException(
                f'Ran into an error, look at the help menu.\nError {exception}')

    try:
        # Run the contianer
        click.echo(f'Running {tag}...')
        if display == 'mesa':
            container = client.containers.run(
                tag, stdin_open=True, tty=True, auto_remove=True,
                network_mode='host',
                name='amp-assv2-scratch',
                environment={
                    'DISPLAY': os.getenv('DISPLAY'),
                    'QT_X11_NO_MITSHM': 1,
                },
                devices=['/dev/dri:/dev/dri'],
                volumes={
                    '/tmp/.X11-unix': {
                        'bind': '/tmp/.X11-unix',
                        'mode': 'rw',
                    },
                },
                privileged=True,
                detach=True)
        # TODO: NVidia settings
        else:
            raise click.ClickException('NVidia config WIP')
        # Attach to the container
        click.echo('Use [<Ctrl> + D] to stop and exit the container.')
        click.secho(
            f'Entering container {container.name}:{container.short_id}.', fg='blue')
        subprocess.run(['bash', '-c', 'xhost +local:docker'])
        subprocess.run(['bash', '-c', f'docker attach {container.id}'])
        subprocess.run(['bash', '-c', 'xhost -local:docker'])
        click.secho('Successfully exited and stopped container.', fg='green')

    except docker.errors.ImageNotFound:
        raise click.ClickException(
            f'Unable to find {tag}, maybe try running with "--build" flag?')
    except Exception as exception:
        raise click.ClickException(
            f'Ran into an error, look at the help menu.\nError {exception}')
