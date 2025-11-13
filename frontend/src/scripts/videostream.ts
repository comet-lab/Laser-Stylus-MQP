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
    const robotBtn = document.getElementById('robot-toggle-container') as HTMLButtonElement;
    const laserBtn = document.getElementById('laser-toggle-container') as HTMLButtonElement;
    const robotStatus = document.getElementById('robotStatus') as HTMLElement;
    const laserStatus = document.getElementById('laserStatus') as HTMLElement;
    const drawBtn = document.getElementById('drawBtn') as HTMLButtonElement;
    const clearBtn = document.getElementById('clearBtn') as HTMLButtonElement;
    const sendBtn = document.getElementById('sendBtn') as HTMLButtonElement;
    const toggleButtons = document.querySelectorAll('#middle-icon-section .icon-btn');

    //Canvas setup
    ctx.font = "48px serif";
    let reader: any = null;

    let isDrawing = false;
    let hasPattern = false;

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

        // 2. Read the *current* state from the UI
        const isCurrentlyOn = getLocalLaserState();

        // 3. Request the *opposite* state
        const desiredNewState = !isCurrentlyOn;

        // 4. Send only the laser state update to backend
        const success = wsHandler.updateState({ isLaserOn: desiredNewState });

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

        isDrawing = !isDrawing;

        if (isDrawing) {
            // Start drawing
            drawBtn.setAttribute('data-state', 'drawing');
            drawBtn.querySelector('.btn-text')!.textContent = 'STOP DRAWING';
            clearBtn.disabled = true;
            sendBtn.disabled = true;

            drawingTracker.enableDrawing();
            drawingTracker.clearDrawing(); // Clear previous drawing
        } else {
            // Stop drawing
            drawBtn.setAttribute('data-state', 'ready');
            drawBtn.querySelector('.btn-text')!.textContent = 'DRAW PATTERN';
            hasPattern = true;
            clearBtn.disabled = false;
            sendBtn.disabled = false;

            drawingTracker.disableDrawing();
        }
    });

    clearBtn.addEventListener('click', () => {
        console.log('Clear button clicked');
        if (!drawingTracker) return;

        // Clear the drawing
        drawingTracker.clearDrawing();

        // Update state
        hasPattern = false;
        clearBtn.disabled = true;
        sendBtn.disabled = true;

        // Reset draw button state if needed
        if (!isDrawing) {
            drawBtn.setAttribute('data-state', 'ready');
            drawBtn.querySelector('.btn-text')!.textContent = 'DRAW PATTERN';
        }
    });


    // Handler for the send button
    sendBtn.addEventListener('click', async () => {
        if (!drawingTracker) return;

        // Show confirmation modal
        if (!confirm('Execute this pattern on the robot? This action cannot be undone.')) {
            return;
        }

        sendBtn.disabled = true;
        clearBtn.disabled = true;

        try {
            console.log('Sending coordinates to robot...');
            const result = await drawingTracker.sendCoordinates();

            // Clear the drawing after successful send
            drawingTracker.clearDrawing();
            hasPattern = false;

            // Reset button states
            drawBtn.setAttribute('data-state', 'ready');
            drawBtn.querySelector('.btn-text')!.textContent = 'DRAW PATTERN';
        }
        catch (e) {
            console.error('Error sending coordinates:', e);
            // Re-enable buttons on error
            if (hasPattern) {
                sendBtn.disabled = false;
                clearBtn.disabled = false;
            }
        }
    });



    toggleButtons.forEach(clickedButton => {
        clickedButton.addEventListener('click', () => {

            // Check if the clicked button is already selected
            const isAlreadySelected = clickedButton.classList.contains('selected');

            // 1. Remove 'selected' from ALL middle buttons
            toggleButtons.forEach(btn => {
                btn.classList.remove('selected');
            });

            // 2. If it wasn't already selected, add the class to it.
            // (If it *was* selected, step 1 already deselected it,
            // which creates the "toggle off" behavior)
            if (!isAlreadySelected) {
                clickedButton.classList.add('selected');
            }
        });
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

    robotBtn.addEventListener('click', () => {
        robotBtn.classList.toggle('active');
    });
});