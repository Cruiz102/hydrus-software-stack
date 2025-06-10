import socket
import uuid
import random
import string
import os

import docker

REPO_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

class DockerManager:
    """Handle creation and removal of SSH-enabled Docker containers."""

    def __init__(self):
        self.client = docker.from_env()

    def _free_port(self) -> int:
        s = socket.socket()
        s.bind(("", 0))
        port = s.getsockname()[1]
        s.close()
        return port

    def create_container(self, user_id: int):
        cid = uuid.uuid4().hex[:8]
        password = "".join(random.choices(string.ascii_letters + string.digits, k=10))
        port = self._free_port()
        env = {
            "PASSWORD_ACCESS": "true",
            "USER_NAME": "dev",
            "USER_PASSWORD": password,
            "PUID": "1000",
            "PGID": "1000",
            "TZ": "UTC",
        }
        container = self.client.containers.run(
            "linuxserver/openssh-server",
            detach=True,
            ports={"2222/tcp": port},
            environment=env,
            volumes={REPO_PATH: {"bind": "/workspace", "mode": "rw"}},
            name=f"user-{user_id}-{cid}",
        )
        return cid, container.id, port, password

    def remove_container(self, docker_id: str) -> None:
        try:
            container = self.client.containers.get(docker_id)
            container.stop()
            container.remove()
        except docker.errors.NotFound:
            pass
