import docker
import os
import subprocess

import click


def write_log_to_file(file_name, log_generator):
    with open(file_name, "w") as f:
        for log in log_generator:
            try:
                f.write(log["stream"])
            except:
                pass


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
@click.option("-b",
              "--build",
              is_flag=True,
              help="Build the frame container, if not up-to-date.")
@click.option(
    "--display",
    default="mesa",
    type=click.Choice(["mesa", "nvidia"], case_sensitive=False),
    help="Choose display driver option.",
)
def devel(build: bool, display: str):
    """Develop with current directory mounted on the container."""
    # Initialize docker client environment
    try:
        client = docker.from_env()
    except Exception as exception:
        raise click.ClickException(
            f"Error: {exception}\nRan into an error, look at the help menu.")
    cwd = os.getcwd()
    tag = "amp-devel:frame-desktop"

    # Make sure current directory is AMP_ASSv2
    if cwd.split("/")[-1] != "AMP_ASSv2":
        raise click.ClickException(
            click.style(
                "Devel only works when executed from the root directory of the AMP_ASSv2 local repo.",
                fg="red",
            ))

    if build:
        subprocess.run(["bash", "-c", "mkdir -p logs"])
        click.echo("Building containers...")

        click.echo("Step 1/2: Building amp-devel:noetic-desktop.")
        try:
            _, logs = client.images.build(
                path=cwd,
                tag="amp-devel:noetic-desktop",
                dockerfile=(cwd + "/docker/desktop.Dockerfile"),
            )
            click.secho(" ---> Finished building amp-devel:noetic-desktop.",
                        fg="yellow")
        except Exception as exception:
            raise click.ClickException(
                f"Error: {exception}\nRan into an error, look at the help menu."
            )
        write_log_to_file("logs/devel-build-desktop.log", logs)
        click.secho(" ---> Finished saving logs.", fg="yellow")

        click.echo(f"Step 2/2: Building {tag}.")
        try:
            _, logs = client.images.build(
                path=cwd,
                tag=tag,
                dockerfile=(cwd + "/docker/frame.Dockerfile"))
            click.secho(f" ---> Finished building {tag}.", fg="yellow")
        except Exception as exception:
            raise click.ClickException(
                f"Error: {exception}\nRan into an error, look at the help menu."
            )
        write_log_to_file("logs/devel-build-frame.log", logs)
        click.secho(" ---> Finished saving logs.", fg="yellow")

    try:
        subprocess.run(["bash", "-c", "mkdir -p amp-devel/build"])
        subprocess.run(["bash", "-c", "mkdir -p amp-devel/devel"])
        # Run the container
        click.echo(f"Running {tag}...")
        if display == "mesa":
            container = client.containers.run(
                tag,
                stdin_open=True,
                tty=True,
                auto_remove=True,
                network_mode="host",
                name="amp-assv2-scratch",
                environment={
                    "DISPLAY": os.getenv("DISPLAY"),
                    "QT_X11_NO_MITSHM": 1,
                },
                devices=["/dev/dri:/dev/dri"],
                volumes={
                    "/tmp/.X11-unix": {
                        "bind": "/tmp/.X11-unix",
                        "mode": "rw",
                    },
                    cwd + "/src": {
                        "bind": "/amp_ws/src",
                        "mode": "rw",
                    },
                    cwd + "/amp-devel/build": {
                        "bind": "/amp_ws/build",
                        "mode": "rw",
                    },
                    cwd + "/amp-devel/devel": {
                        "bind": "/amp_ws/devel",
                        "mode": "rw",
                    },
                },
                privileged=True,
                detach=True,
            )
        # TODO: NVidia settings
        else:
            raise click.ClickException("NVidia config WIP")
        # Attach to the container
        click.echo("Use [<Ctrl> + D] to stop and exit the container.")
        click.secho("Note: Rosdep packages are not installed in devel",
                    fg="yellow")
        click.secho('Run "apt update && rosdep install --from-paths src -iry"',
                    fg="yellow")
        click.secho(
            f"Entering container {container.name}:{container.short_id}.",
            fg="blue")
        subprocess.run(["bash", "-c", "xhost +"])
        subprocess.run(["bash", "-c", f"docker attach {container.id}"])
        subprocess.run(["bash", "-c", "xhost -"])
        click.secho("Successfully exited and stopped container.", fg="green")
    except docker.errors.ImageNotFound:
        raise click.ClickException(
            f'Unable to find {tag}, maybe try running with "--build" flag?')
    except Exception as exception:
        raise click.ClickException(
            f"Error: {exception}\nRan into an error, look at the help menu.")


@cli.command()
@click.option("-b",
              "--build",
              is_flag=True,
              help="Build the corresponding containers.")
@click.option(
    "--display",
    default="mesa",
    show_default=True,
    type=click.Choice(["mesa", "nvidia"], case_sensitive=False),
    help="Choose display driver option.",
)
def scratch(build: bool, display: str):
    """Develop on a fully isolated container."""
    # Initialize docker client environment
    try:
        client = docker.from_env()
    except Exception as exception:
        raise click.ClickException(
            f"Error: {exception}\nRan into an error, look at the help menu.")
    cwd = os.getcwd()

    tag = f"amp-devel:{display}-build"
    dockerfile = cwd + f"/docker/{display}.Dockerfile"

    # Build container
    if build:
        # Make sure current directory is AMP_ASSv2
        if cwd.split("/")[-1] != "AMP_ASSv2":
            raise click.ClickException(
                click.style(
                    "Build only works inside root directory of the AMP_ASSv2 local repository.",
                    fg="red",
                ))

        subprocess.run(["bash", "-c", "mkdir -p logs"])
        click.echo("Building containers...")

        click.echo("Step 1/3: Building amp-devel:noetic-desktop.")
        try:
            _, logs = client.images.build(
                path=cwd,
                tag="amp-devel:noetic-desktop",
                dockerfile=(cwd + "/docker/desktop.Dockerfile"),
            )
            click.secho(" ---> Finished building amp-devel:noetic-desktop.",
                        fg="yellow")
        except Exception as exception:
            raise click.ClickException(
                f"Error: {exception}\nRan into an error, look at the help menu."
            )
        write_log_to_file("logs/scratch-build-desktop.log", logs)
        click.secho(" ---> Finished saving logs.", fg="yellow")

        click.echo("Step 2/3: Building amp-devel:frame-desktop.")
        try:
            _, logs = client.images.build(
                path=cwd,
                tag="amp-devel:frame-desktop",
                dockerfile=(cwd + "/docker/frame.Dockerfile"),
            )
            click.secho(" ---> Finished building amp-devel:frame-desktop.",
                        fg="yellow")
        except Exception as exception:
            raise click.ClickException(
                f"Error: {exception}\nRan into an error, look at the help menu."
            )
        write_log_to_file("logs/scratch-build-frame.log", logs)
        click.secho(" ---> Finished saving logs.", fg="yellow")

        click.echo(f"Step 3/3: Building {tag}.")
        try:
            _, logs = client.images.build(path=cwd,
                                          tag=tag,
                                          dockerfile=dockerfile)
            click.secho(f" ---> Finished building {tag}.", fg="yellow")
        except Exception as exception:
            raise click.ClickException(
                f"Error: {exception}\nRan into an error, look at the help menu."
            )
        write_log_to_file(f"logs/scratch-build-{display}.log", logs)
        click.secho(" ---> Finished saving logs.", fg="yellow")

    try:
        # Run the container
        click.echo(f"Running {tag}...")
        if display == "mesa":
            container = client.containers.run(
                tag,
                stdin_open=True,
                tty=True,
                auto_remove=True,
                network_mode="host",
                name="amp-assv2-scratch",
                environment={
                    "DISPLAY": os.getenv("DISPLAY"),
                    "QT_X11_NO_MITSHM": 1,
                },
                devices=["/dev/dri:/dev/dri"],
                volumes={
                    "/tmp/.X11-unix": {
                        "bind": "/tmp/.X11-unix",
                        "mode": "rw",
                    },
                },
                privileged=True,
                detach=True,
            )
        # TODO: NVidia settings
        else:
            raise click.ClickException("NVidia config WIP")
        # Attach to the container
        click.echo("Use [<Ctrl> + D] to stop and exit the container.")
        click.secho(
            f"Entering container {container.name}:{container.short_id}.",
            fg="blue")
        subprocess.run(["bash", "-c", "xhost +"])
        subprocess.run(["bash", "-c", f"docker attach {container.id}"])
        subprocess.run(["bash", "-c", "xhost -"])
        click.secho("Successfully exited and stopped container.", fg="green")

    except docker.errors.ImageNotFound:
        raise click.ClickException(
            f'Unable to find {tag}, maybe try running with "--build" flag?')
    except Exception as exception:
        raise click.ClickException(
            f"Error: {exception}\nRan into an error, look at the help menu.")
