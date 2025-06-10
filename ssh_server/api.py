from typing import List

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

from .database import Database
from .docker_manager import DockerManager

app = FastAPI()

db = Database()
docker_manager = DockerManager()

class UserRequest(BaseModel):
    name: str | None = None
    email: str
    password: str

class ContainerInfo(BaseModel):
    id: str
    port: int
    password: str

@app.post("/register")
def register(req: UserRequest):
    if not req.name:
        raise HTTPException(status_code=400, detail="Name required")
    if not db.add_user(req.name, req.email, req.password):
        raise HTTPException(status_code=400, detail="Email already registered")
    return {"message": "registered"}

@app.post("/login")
def login(req: UserRequest):
    user_id = db.verify_user(req.email, req.password)
    if not user_id:
        raise HTTPException(status_code=401, detail="Invalid credentials")
    return {"message": "success"}

@app.post("/container", response_model=ContainerInfo)
def create_container(req: UserRequest):
    user_id = db.verify_user(req.email, req.password)
    if not user_id:
        raise HTTPException(status_code=401, detail="Invalid credentials")
    cid, docker_id, port, password = docker_manager.create_container(user_id)
    db.add_container(cid, user_id, docker_id, port, password)
    return ContainerInfo(id=cid, port=port, password=password)

@app.get("/container", response_model=List[ContainerInfo])
def list_containers(email: str, password: str):
    user_id = db.verify_user(email, password)
    if not user_id:
        raise HTTPException(status_code=401, detail="Invalid credentials")
    rows = db.list_containers(user_id)
    return [ContainerInfo(id=r[0], port=r[2], password=r[3]) for r in rows]

@app.delete("/container/{cid}")
def delete_container(cid: str, email: str, password: str):
    user_id = db.verify_user(email, password)
    if not user_id:
        raise HTTPException(status_code=401, detail="Invalid credentials")
    rows = db.list_containers(user_id)
    for r in rows:
        if r[0] == cid:
            docker_manager.remove_container(r[1])
            db.delete_container(user_id, cid)
            return {"message": "deleted"}
    raise HTTPException(status_code=404, detail="Not found")
