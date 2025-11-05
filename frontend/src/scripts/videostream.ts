import { DrawingTracker } from './classes/DrawingTracker';

declare global {
    interface Window {
        MediaMTXWebRTCReader: any;
    }
}

//Wait for the window to load
window.addEventListener('load', () => {
    //Get all components
    const video = document.getElementById('video') as HTMLVideoElement;
    const canvas = document.getElementById('canvas') as HTMLCanvasElement;
    const ctx = canvas.getContext("2d")!;
    const ws = new WebSocket(`ws://localhost:443/ws/${import.meta.env.VITE_UI_WEBSOCKET_NAME}`);
    const drawBtn = document.getElementById('drawBtn') as HTMLButtonElement;
    const sendBtn = document.getElementById('sendBtn') as HTMLButtonElement;
    const coordinateOutput = document.getElementById("coordinateOutput") as HTMLElement;

    // WebSocket event handlers
    ws.onopen = () => console.log("Connected to WebSocket\n");
    ws.onmessage = (event) => {
        console.log("Received from backend:", event.data);
        coordinateOutput.textContent = `Response: ${event.data}`;
    };
    ws.onclose = () => console.log("Disconnected\n");
    ws.onerror = (error) => console.log("WebSocket error: " + error + "\n");

    //Canvas setup
    ctx.font = "48px serif";
    let reader: any = null;

    // Initialize drawing tracker
    let drawingTracker: DrawingTracker | null = null;

    //Update canvas with video frame
    const updateCanvas = (now: DOMHighResTimeStamp, metadata: VideoFrameCallbackMetadata) => {
        ctx.drawImage(video, 0, 0, canvas.width, canvas.height);

        // Draw the drawing overlay on top
        if (drawingTracker) {
            drawingTracker.drawOnMainCanvas();
        }

        video.requestVideoFrameCallback(updateCanvas);
    };

    // Function to set a message on the canvas
    const setMessage = (str: string) => {
        ctx.fillText(str, 10, 50);
    };

    // Setup video and canvas
    video.muted = true;
    video.autoplay = true;
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
    setMessage("Loading stream");

    //Render with MediaMTX WebRTC Reader
    reader = new window.MediaMTXWebRTCReader({
        url: new URL('http://localhost:8889/mystream/whep'),
        onError: (err: string) => {
            setMessage(err);
        },
        onTrack: (evt: RTCTrackEvent) => {
            video.srcObject = evt.streams[0];
            video.requestVideoFrameCallback(updateCanvas);

            // Initialize drawing tracker after video is ready
            drawingTracker = new DrawingTracker(canvas, video, ws);
            
            // Initially disable send button
            sendBtn.disabled = true;
        },
    });

    // Handler for the draw button
    drawBtn.addEventListener('click', () => {
        if (!drawingTracker) return;

        if (drawingTracker.isDrawingEnabled()) {
            // Stop drawing
            drawingTracker.disableDrawing();
            drawBtn.textContent = 'Start Drawing';
            sendBtn.disabled = false; // Enable send button
        } else {
            // Start drawing
            drawingTracker.enableDrawing();
            drawingTracker.clearDrawing(); // Clear previous drawing
            drawBtn.textContent = 'Stop Drawing';
            sendBtn.disabled = true; // Disable send while drawing
            coordinateOutput.textContent = ''; // Clear previous output
        }
    });

    // Handler for the send button
    sendBtn.addEventListener('click', () => {
        if (!drawingTracker) return;

        drawingTracker.sendCoordinates((response) => {
            coordinateOutput.textContent = `Response: ${response}`;
        });

        // Clear drawing after sending
        drawingTracker.clearDrawing();
    });

    // Window event handlers
    window.addEventListener('beforeunload', () => {
        if (reader !== null) {
            reader.close();
        }
    });

    //Window resize logic
    window.addEventListener('resize', () => {
        canvas.width = window.innerWidth;
        canvas.height = window.innerHeight;
        console.log(canvas.width);
        console.log(canvas.height);

        // Update drawing canvas size
        if (drawingTracker) {
            drawingTracker.updateCanvasSize(canvas.width, canvas.height);
        }
    });
});