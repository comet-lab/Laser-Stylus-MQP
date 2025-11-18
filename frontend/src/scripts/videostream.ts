import { DrawingTracker } from './drawing/DrawingTracker';
import { ShapeType } from './drawing/types';
import { WebSocketHandler, WebSocketMessage } from './classes/WebSocketHandler';

declare global {
    interface Window {
        MediaMTXWebRTCReader: any;
    }
}

//Wait for the window to load
window.addEventListener('load', () => {
    //Get all components
    const settingsBtn = document.getElementById('settingsBtn') as HTMLButtonElement;
    const settingsPopup = document.getElementById('settingsPopup') as HTMLElement;
    const overlay = document.getElementById('overlay') as HTMLElement;
    const settingsCloseBtn = document.getElementById('settingsCloseBtn') as HTMLButtonElement;

    const preparePopup = document.getElementById('preparePopup') as HTMLElement;
    const prepareCloseBtn = document.getElementById('prepareCloseBtn') as HTMLButtonElement;
    const prepareCancelBtn = document.getElementById('prepareCancelBtn') as HTMLButtonElement;
    const executeBtn = document.getElementById('executeBtn') as HTMLButtonElement;

    const video = document.getElementById('video') as HTMLVideoElement;
    const canvas = document.getElementById('canvas') as HTMLCanvasElement;
    const ctx = canvas.getContext("2d")!;
    const robotBtn = document.getElementById('robot-toggle-container') as HTMLButtonElement;
    const laserBtn = document.getElementById('laser-toggle-container') as HTMLButtonElement;
    const drawBtn = document.getElementById('drawBtn') as HTMLButtonElement;
    const prepareBtn = document.getElementById('prepareBtn') as HTMLButtonElement;
    
    // Shape buttons
    const penBtn = document.getElementById('penBtn') as HTMLButtonElement;
    const squareBtn = document.getElementById('squareBtn') as HTMLButtonElement;
    const circleBtn = document.getElementById('circleBtn') as HTMLButtonElement;
    const triangleBtn = document.getElementById('triangleBtn') as HTMLButtonElement;
    const lineBtn = document.getElementById('lineBtn') as HTMLButtonElement;
    const toggleButtons = document.querySelectorAll('#middle-icon-section .icon-btn');

    const sidebarButtons: NodeListOf<HTMLButtonElement> = document.querySelectorAll('.settings-sidebar .sidebar-btn');
    const settingsPanels: NodeListOf<HTMLElement> = document.querySelectorAll('.settings-main .settings-panel');

    let laserConfirmationTimeout: number | null = null;
    let drawingState: 'idle' | 'complete' = 'idle';
    let selectedShape: ShapeType | null = null;

    //Canvas setup
    ctx.font = "48px serif";
    let reader: any = null;

    // Initialize drawing tracker
    let drawingTracker: DrawingTracker | null = null;
    const wsHandler = new WebSocketHandler(null);

    // --- Helper Functions ---

    const openSettings = (): void => {
        if (drawingState === 'complete') {
            drawingTracker?.clearDrawing();
            drawingState = 'idle';
            updateDrawButtonState();
        }

        // Disable all controls
        [drawBtn, prepareBtn, robotBtn, laserBtn].forEach(btn => {
            btn.disabled = true;
        });

        //Disable laser
        changeLaserState(false);

        //DISABLE ROBOT WHEN IMPLEMENTED

        settingsPopup.classList.add('active');
        overlay.classList.add('active');
    };

    const closeSettings = (): void => {
        settingsPopup.classList.remove('active');
        overlay.classList.remove('active');

        [drawBtn, robotBtn, laserBtn].forEach(btn => {
            btn.disabled = false;
        });
    };

    settingsBtn.addEventListener('click', openSettings);
    settingsCloseBtn.addEventListener('click', closeSettings);
    

    const openPrepareMenu = (): void => {
        overlay.classList.add('active');
        preparePopup.classList.add('active');
    };

    const closePrepareMenu = (): void => {
        overlay.classList.remove('active');
        preparePopup.classList.remove('active');
    };

    prepareBtn.addEventListener('click', openPrepareMenu);
    prepareCloseBtn.addEventListener('click', closePrepareMenu);
    prepareCancelBtn.addEventListener('click', () => {
        closePrepareMenu();
    });

    overlay.addEventListener('click', () => {
        if (settingsPopup.classList.contains('active')) {
            closeSettings();
        }
        if (preparePopup.classList.contains('active')) {
            closePrepareMenu();
        }
    });

    

    sidebarButtons.forEach((button: HTMLButtonElement) => {
        button.addEventListener('click', () => {
            const targetId: string | null = button.getAttribute('data-target');
            if (!targetId) return;

            // Remove 'active' from all buttons and panels
            sidebarButtons.forEach((btn: HTMLButtonElement) => btn.classList.remove('active'));
            settingsPanels.forEach((panel: HTMLElement) => panel.classList.remove('active'));

            // Add 'active' to the clicked button and target panel
            button.classList.add('active');
            const targetPanel: HTMLElement | null = document.getElementById(targetId);
            if (targetPanel) {
                targetPanel.classList.add('active');
            }
        });
    });

    function updateDrawButtonState() {
        const btnText = drawBtn.querySelector('.btn-text')!;

        switch (drawingState) {
            case 'idle':
                drawBtn.setAttribute('data-state', 'ready');
                btnText.textContent = 'CLEAR PATH';
                drawBtn.disabled = true;
                executeBtn.disabled = true;
                prepareBtn.disabled = true;
                break;
            case 'complete':
                drawBtn.setAttribute('data-state', 'clear');
                btnText.textContent = 'CLEAR PATH';
                drawBtn.disabled = false;
                executeBtn.disabled = false;
                prepareBtn.disabled = false;
                break;
        }
    }

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

            // Initially disable buttons
            executeBtn.disabled = true;
            prepareBtn.disabled = true;
            drawBtn.disabled = true;
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
        changeLaserState(desiredNewState);
        // Note: Button will be re-enabled when we receive the state update from backend
        // via the onStateUpdate -> syncUiToState flow
    });

    function changeLaserState(newState: boolean) {
        const success = wsHandler.updateState({ isLaserOn: newState });

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
    }

    // Handler for the clear button (formerly draw button)
    drawBtn.addEventListener('click', () => {
        console.log('Clear button clicked');
        if (!drawingTracker) return;

        if (drawingState === 'complete') {
            // Clear the drawing
            drawingTracker.clearDrawing();
            drawingState = 'idle';
            updateDrawButtonState();
            
            // Deselect all shape buttons
            toggleButtons.forEach(btn => btn.classList.remove('selected'));
            selectedShape = null;
        }
    });

    // Handler for the execute button
    executeBtn.addEventListener('click', async () => {
        if (!drawingTracker) return;

        executeBtn.disabled = true;
        prepareBtn.disabled = true;

        try {
            console.log('Sending coordinates to robot...');
            const result = await drawingTracker.sendCoordinates();

            // Clear the drawing after successful send
            drawingTracker.clearDrawing();
            drawingState = 'idle';
            updateDrawButtonState();
            closePrepareMenu();
            
            // Deselect all shape buttons
            toggleButtons.forEach(btn => btn.classList.remove('selected'));
            selectedShape = null;
        }
        catch (e) {
            console.error('Error sending coordinates:', e);
            // Re-enable button on error
            if (drawingState === 'complete') {
                executeBtn.disabled = false;
                prepareBtn.disabled = false;
            }
        }
    });

    // Shape button handlers
    function handleShapeSelection(button: HTMLButtonElement, shape: ShapeType) {
        // Check if the clicked button is already selected
        const isAlreadySelected = button.classList.contains('selected');

        // Remove 'selected' from all buttons
        toggleButtons.forEach(btn => btn.classList.remove('selected'));

        if (isAlreadySelected) {
            // Deselect
            selectedShape = null;
            if (drawingTracker) {
                drawingTracker.disableDrawing();
            }
        } else {
            // Select new shape
            button.classList.add('selected');
            selectedShape = shape;
            
            if (drawingTracker) {
                // Clear any existing drawing when selecting a new shape
                drawingTracker.clearDrawing();
                drawingState = 'idle';
                updateDrawButtonState();
                
                // Enable drawing with the selected shape
                drawingTracker.setShapeType(shape);
                drawingTracker.enableDrawing(() => {
                    // Callback when shape is complete
                    drawingState = 'complete';
                    updateDrawButtonState();
                });
            }
        }
    }

    penBtn.addEventListener('click', () => handleShapeSelection(penBtn, 'freehand'));
    squareBtn.addEventListener('click', () => handleShapeSelection(squareBtn, 'square'));
    circleBtn.addEventListener('click', () => handleShapeSelection(circleBtn, 'circle'));
    triangleBtn.addEventListener('click', () => handleShapeSelection(triangleBtn, 'triangle'));
    lineBtn.addEventListener('click', () => handleShapeSelection(lineBtn, 'line'));

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
