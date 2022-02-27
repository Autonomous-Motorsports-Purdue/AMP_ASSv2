import docker
import os
import subprocess

import click


@click.group()
def cli():
    """This is the official AMP Software Sub-Team CLI Tool that helps you 
    develop on AMP_ASSv2 using docker containers in your current environment.

    In order to run it, you must be in the root directory of the AMP_ASSv2 repo
    in order for docker files to build properly.
    """
    pass


@cli.command()
@click.option('-b', '--build', is_flag=True)
@click.option('--display', default='mesa',
              type=click.Choice(['mesa', 'nvidia'], case_sensitive=False))
def devel(build: bool, display: str):
    """Develop with current directory mounted on the container.
    """
    print(display)
    click.echo('Successfully exited the container')


@cli.command()
@click.option('-b', '--build', is_flag=True,
              help='Build a new container.')
@click.option('--display', default='mesa', show_default=True,
              type=click.Choice(['mesa', 'nvidia'], case_sensitive=False),
              help='Choose display option.')
def scratch(build: bool, display: str):
    """Develop on a fully isolated container.
    """
    # Initialize docker client environment
    client = docker.from_env()
    cwd = os.getcwd()

    # Make sure current directory is AMP_ASSv2
    if cwd.split('/')[-1] != 'AMP_ASSv2':
        raise click.ClickException(click.style(
            'AMP CLI only works inside root directory of the AMP_ASSv2 local repository', fg='red'))

    tag = f'amp-ass:{display}'
    dockerfile = cwd + f'/docker/{display}.Dockerfile'

    # Build container
    if build:
        click.echo('Building containers...')
        click.echo('Step 1/2: Building amp-base:default')
        try:
            _ = client.images.build(
                path=cwd, tag='amp-base:default',
                dockerfile=(cwd+'/docker/base.Dockerfile'))
            click.secho(
                f' ---> Finished building amp-base:default', fg='yellow')
        except Exception as exception:
            raise click.ClickException(exception)

        click.echo(f'Step 2/2: Building {tag}')
        try:
            _ = client.images.build(
                path=cwd, tag=tag, dockerfile=dockerfile)
            click.secho(f' ---> Finished building {tag}', fg='yellow')
        except Exception as exception:
            raise click.ClickException(exception)

    try:
        # Run the contianer
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
                        'mode': 'rw'
                    }
                },
                privileged=True,
                detach=True)
        else:
            container = client.containers.run(
                tag, stdin_open=True, tty=True, auto_remove=True,
                network_mode='host',
                environment={
                    'DISPLAY': os.getenv('DISPLAY'),
                    'QT_X11_NO_MITSHM': 1,
                },
                devices=['/dev/dri:/dev/dri'],
                volumes={
                    '/tmp/.X11-unix': {
                        'bind': '/tmp/.X11-unix',
                        'mode': 'rw'
                    }
                },
                privileged=True,
                detach=True)
        click.secho(
            f'Entering container {container.name}:{container.short_id}', fg='blue')

        # Attach to the container
        subprocess.run(['docker', 'attach', container.id])
        click.secho('Successfully exited and stopped container', fg='green')
    except Exception as exception:
        raise click.ClickException(exception)
