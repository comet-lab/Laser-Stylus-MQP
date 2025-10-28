# Frontend (Vite + TypeScript)

This folder contains a minimal Vite + TypeScript frontend used for development and served in Docker.

How to run locally (without Docker):

1. cd frontend
2. npm install
3. npm run dev

How to build (Docker):

docker-compose up --build

The frontend will be available at http://localhost:3000 and calls the backend at http://localhost:443/media/mystream when you click the "Connect to backend" button.
