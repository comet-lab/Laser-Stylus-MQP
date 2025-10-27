from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()

#app.mount("/static", StaticFiles(directory="static"), name="static")

# Allow the frontend (served on localhost:3000) to call backend APIs during development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/media/{stream_name}")
def read_item(stream_name: str):
    return {"stream_url": "http://localhost:8889/mystream"}