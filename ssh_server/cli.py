from .database import Database
from .docker_manager import DockerManager


def cli() -> None:
    """Simple interactive command line interface."""
    db = Database()
    docker_manager = DockerManager()

    print("=== SSH Docker Server CLI ===")
    choice = input("1) Register\n2) Login\nSelect: ")
    if choice == "1":
        name = input("Name: ")
        email = input("Email: ")
        pwd = input("Password: ")
        if db.add_user(name, email, pwd):
            print("Registered successfully")
        else:
            print("Email already registered")
        return
    elif choice != "2":
        return

    email = input("Email: ")
    pwd = input("Password: ")
    user_id = db.verify_user(email, pwd)
    if not user_id:
        print("Invalid credentials")
        return

    while True:
        print("\n1) Create instance\n2) List instances\n3) Delete instance\n4) Exit")
        opt = input("Select: ")
        if opt == "1":
            cid, docker_id, port, passwd = docker_manager.create_container(user_id)
            db.add_container(cid, user_id, docker_id, port, passwd)
            print(f"Container {cid} created")
            print(f"SSH command: ssh dev@<host> -p {port}")
            print(f"Password: {passwd}")
        elif opt == "2":
            for r in db.list_containers(user_id):
                print(f"{r[0]} -> port {r[2]}")
        elif opt == "3":
            tid = input("Container id: ")
            found = False
            for r in db.list_containers(user_id):
                if r[0] == tid:
                    docker_manager.remove_container(r[1])
                    db.delete_container(user_id, tid)
                    print("Deleted")
                    found = True
                    break
            if not found:
                print("Not found")
        else:
            break
