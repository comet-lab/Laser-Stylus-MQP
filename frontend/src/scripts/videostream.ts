import { DrawingTracker } from './classes/DrawingTracker';
import { WebSocketHandler, WebSocketMessage } from './classes/WebSocketHandler';

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
    const robotBtn = document.getElementById('robotBtn') as HTMLButtonElement;
    const laserBtn = document.getElementById('laser-toggle-container') as HTMLButtonElement;
    const robotStatus = document.getElementById('robotStatus') as HTMLElement;
    const laserStatus = document.getElementById('laserStatus') as HTMLElement;
    const drawBtn = document.getElementById('drawBtn') as HTMLButtonElement;
    const sendBtn = document.getElementById('sendBtn') as HTMLButtonElement;
    const coordinateOutput = document.getElementById("status-panel-left") as HTMLElement;

    let laserConfirmationTimeout: number | null = null;

    //Canvas setup
    ctx.font = "48px serif";
    let reader: any = null;

    // Initialize drawing tracker
    let drawingTracker: DrawingTracker | null = null;
    const wsHandler = new WebSocketHandler(null);

    // --- Helper Functions ---

    /** Reads the laser's *current state* from the button's class or data attribute */
    function getLocalLaserState(): boolean {
        return laserBtn.classList.contains('active');
    }

    /**
     * Single function for updating the UI based on backend state.
     * Called by the WebSocket handler whenever state updates arrive.
     */
    function syncUiToState(state: Partial<WebSocketMessage>) {
        // Update laser button and status (only if laser state is present)
        if (state.isLaserOn !== undefined) {
            //Clear timeout if you get a response from the robot
            if (laserConfirmationTimeout) {
                clearTimeout(laserConfirmationTimeout);
                laserConfirmationTimeout = null;
            }

            const newLaserState = !!state.isLaserOn;
            // Update button text and class
            laserBtn.classList.toggle('active', newLaserState)
            // Update status display
            laserStatus.textContent = `Laser: ${newLaserState ? 'ON' : 'OFF'}`;
            // Re-enable button after state sync
            laserBtn.style.pointerEvents = 'auto';
        }

        // Update robot status if present
        // (You can add robot state handling here later if needed)
    }

    // --- WebSocket Setup ---

    // Assign the callback *before* connecting.
    // This function will run every time the backend broadcasts a new state.
    wsHandler.onStateUpdate = (newState: WebSocketMessage) => {
        console.log('State update received:', newState);
        syncUiToState(newState);
    };

    wsHandler.connect();
    //(window as any).wsHandler = wsHandler;

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
        url: new URL(`http://${window.location.hostname}:8889/mystream/whep`),
        onError: (err: string) => {
            setMessage(err);
        },
        onTrack: (evt: RTCTrackEvent) => {
            video.srcObject = evt.streams[0];
            video.requestVideoFrameCallback(updateCanvas);

            // Initialize drawing tracker after video is ready
            drawingTracker = new DrawingTracker(canvas, video, `http://${window.location.hostname}:443`);

            // Initially disable send button
            sendBtn.disabled = true;
        },
    });

    // --- Event Listeners ---

    // Handler for laser button - request a *toggle*
    laserBtn.addEventListener('click', () => {
        // 1. Disable button immediately to prevent multiple clicks
        laserBtn.style.pointerEvents = 'none';

        //Clear old timeouts
        if (laserConfirmationTimeout) {
            clearTimeout(laserConfirmationTimeout);
        }

        // 2. Read the *current* state from the UI
        const isCurrentlyOn = getLocalLaserState();

        // 3. Request the *opposite* state
        const desiredNewState = !isCurrentlyOn;

        // 4. Send only the laser state update to backend
        const success = wsHandler.updateState({ isLaserOn: desiredNewState });

        // 5. Set timeout to re-enable the button if no response received
        if (success) {
            laserConfirmationTimeout = setTimeout(() => {
                console.error("No confirmation from robot. Resetting UI.");
                laserBtn.style.pointerEvents = 'auto'; // Re-enable button on timeout, don't change state
            }, 2000); // 2-second timeout
        }

        if (!success) {
            // If send failed, re-enable the button
            laserBtn.style.pointerEvents = 'auto';
            console.error('Failed to send laser state update');
        }
        // Note: Button will be re-enabled when we receive the state update from backend
        // via the onStateUpdate -> syncUiToState flow
    });

    // Handler for the draw button
    drawBtn.addEventListener('click', () => {
        console.log('Draw button clicked');
        if (!drawingTracker) return;

        if (drawingTracker.isDrawingEnabled()) {
            // Stop drawing
            drawingTracker.disableDrawing();
            drawBtn.textContent = 'Start';
            sendBtn.disabled = false; // Enable send button
        } else {
            // Start drawing
            drawingTracker.enableDrawing();
            drawingTracker.clearDrawing(); // Clear previous drawing
            drawBtn.textContent = 'Stop';
            sendBtn.disabled = true; // Disable send while drawing
            coordinateOutput.textContent = ''; // Clear previous output
        }
    });

    // Handler for the send button
    sendBtn.addEventListener('click', async () => {
        if (!drawingTracker) return;

        sendBtn.disabled = true;
        coordinateOutput.textContent = 'Sending coordinates...';

        try {
            const result = await drawingTracker.sendCoordinates();

            if (result) {
                coordinateOutput.textContent = `Sent ${result.pixel_count} pixels. Server response: ${result.message}`;
            }
            else {
                coordinateOutput.textContent = 'No pixels to send.';
            }
            drawingTracker.clearDrawing();
        }
        catch (e) {
            console.error('Error sending coordinates:', e);
            coordinateOutput.textContent = 'Error sending coordinates: ' + (e instanceof Error ? e.message : String(e));
        }
        finally {
            sendBtn.disabled = false;
        }
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

    canvas.addEventListener('click', function (event) {
        if (drawingTracker?.isDrawingEnabled()) {
            return;
        }

        console.log(event.clientX / canvas.width * video.videoWidth);
        console.log(event.clientY / canvas.height * video.videoHeight);

        let x = event.clientX / canvas.width * video.videoWidth;
        let y = event.clientY / canvas.height * video.videoHeight;

        const data = {
            x: x,
            y: y
        };
        wsHandler.updateState(data);
    });
});