# SSH Docker Server

This utility launches short-lived Docker containers running an SSH server with access to the repository contents. Users can create containers via a REST API or an interactive command line interface.

## Prerequisites

- Docker and the ability to run containers.
- Python 3.10 or later with `pip`.

## Installation

Clone the repository and install the dependencies:

```bash
git clone https://github.com/Rumarino-Team/hydrus-software-stack.git
cd hydrus-software-stack
pip install -r requirements.txt
```

## Starting the server

Run the API server (default host `0.0.0.0`, port `8000`):

```bash
python ssh_docker_server.py
```

To use the interactive CLI instead of the API, pass `--cli`:

```bash
python ssh_docker_server.py --cli
```

## Using the CLI

1. Choose **Register** to create an account or **Login** if you already have one.
2. After logging in, select an option:
   - **Create instance** – starts an SSH container and prints the connection details.
   - **List instances** – shows your current containers and their SSH ports.
   - **Delete instance** – stops and removes a container by its ID.
3. Connect to the container with the command printed, e.g.:

```bash
ssh dev@<server-host> -p <port>
```

Use the password that the CLI displays when the container is created.

## REST API

When the server is run without `--cli`, a FastAPI application is available with the following endpoints:

- `POST /register` – create a user. JSON body `{"name": "Alice", "email": "user@example.com", "password": "secret"}`.
- `POST /login` – verify credentials.
- `POST /container` – create a container. Body must include `email` and `password`. Returns the container ID, SSH port and generated password.
- `GET /container` – list containers for the given `email` and `password` query parameters.
- `DELETE /container/{cid}` – remove a container.

The API uses a simple SQLite database (`ssh_server/users.db`) to store users and container mappings.

## Connecting to the container

Each container runs the `linuxserver/openssh-server` image with the repository mounted at `/workspace`. After creating a container, connect via:

```bash
ssh dev@<server-host> -p <port>
```

Replace `<server-host>` with the hostname or IP address of the machine running the server and `<port>` with the provided port number. Authenticate with the generated password. The repository will be available under `/workspace` inside the container.

## Stopping the server

Press `Ctrl+C` in the terminal where the server is running. Any containers you started remain running until deleted via the CLI/API.
