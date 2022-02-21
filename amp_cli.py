import subprocess as sp

import click


@click.group()
def cli():
    """This is the official AMP Software Sub-Team CLI Tool that helps you 
    develop on AMP_ASSv2 using docker containers in your current environment.

    Currently WIP
    """
    pass


@cli.command()
@click.option('-b', '--build', is_flag=True)
@click.option('-d', '--display', default='mesa',
              type=click.Choice(['mesa', 'nvidia'], case_sensitive=False))
def devel(build, display):
    """Develop with the AMP_ASSv2 directory mounted on the container.
    """
    print(display)
    click.echo('Successfully exited the container')


@cli.command()
@click.option('-b', '--build', is_flag=True)
@click.option('-d', '--display', default='mesa',
              type=click.Choice(['mesa', 'nvidia'], case_sensitive=False))
def scratch(build, display):
    """Develop on a fully isolated container.
    """
    DOCKER_ARGS = ['docker', 'run', '-it', '--rm', '--net=host',
                   '--env="DISPLAY=$DISPLAY"', '--env="QT_X11_NO_MITSHM=1"',
                   '--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"',
                   '--privileged', 'amp-ass:dev']
    if build:
        sp.run(['docker', 'build', '--file',
               './docker/amd64.Dockerfile', '--tag', 'amp-ass:dev', '.'])
        print('run build')
    if display == 'mesa':
        DOCKER_ARGS.insert(9, '--device="/dev/dri:/dev/dri"')
    elif display == 'nvidia':
        DOCKER_ARGS.insert(9, '--env="NVIDIA_DRIVER_CAPABILITIES=all"') 
        DOCKER_ARGS.insert(9, '--gpus all')
    print(DOCKER_ARGS)
    sp.run(['xhost', '+', 'local:docker'])
    sp.run(DOCKER_ARGS)
    click.echo('Successfully exited the container')
