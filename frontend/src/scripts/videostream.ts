import { DrawingTracker } from './drawing/DrawingTracker';
import { ShapeType } from './drawing/types';
import { WebSocketHandler, WebSocketMessage } from './classes/WebSocketHandler';

declare global {
    interface Window {
        MediaMTXWebRTCReader: any;
    }
}

// Wait for the window to load
window.addEventListener('load', () => {
    // --- 1. Get DOM Elements ---
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
    const clearBtn = document.getElementById('clearBtn') as HTMLButtonElement;
    const prepareBtn = document.getElementById('prepareBtn') as HTMLButtonElement;

    const processingModeSwitch = document.getElementById('processing-mode') as HTMLInputElement;
    const batchUiElements = document.querySelectorAll('.batch-ui');
    const statusControlValue = document.querySelector('.status-value.status-batch') as HTMLElement;

    const penBtn = document.getElementById('penBtn') as HTMLButtonElement;
    const squareBtn = document.getElementById('squareBtn') as HTMLButtonElement;
    const circleBtn = document.getElementById('circleBtn') as HTMLButtonElement;
    const triangleBtn = document.getElementById('triangleBtn') as HTMLButtonElement;
    const lineBtn = document.getElementById('lineBtn') as HTMLButtonElement;
    const toggleButtons = document.querySelectorAll('#middle-icon-section .icon-btn');

    const sidebarButtons: NodeListOf<HTMLButtonElement> = document.querySelectorAll('.settings-sidebar .sidebar-btn');
    const settingsPanels: NodeListOf<HTMLElement> = document.querySelectorAll('.settings-main .settings-panel');

    // --- 2. State Variables ---
    let laserConfirmationTimeout: number | null = null;
    let drawingState: 'idle' | 'complete' = 'idle';
    let selectedShape: ShapeType | null = null;
    let reader: any = null;
    let drawingTracker: DrawingTracker | null = null;

    // Real-Time State
    let isRealTimeDrawing = false;
    let latestRealTimePos: { x: number, y: number } | null = null;

    // Initialize WebSocket
    const wsHandler = new WebSocketHandler(null);

    // --- 3. UI Helpers ---

    const openSettings = (): void => {
        if (drawingState === 'complete') {
            drawingTracker?.clearDrawing();
            drawingState = 'idle';
            updateDrawButtonState();
        }
        [clearBtn, prepareBtn, robotBtn, laserBtn].forEach(btn => btn.disabled = true);
        changeLaserState(false);
        settingsPopup.classList.add('active');
        overlay.classList.add('active');
    };

    const closeSettings = (): void => {
        settingsPopup.classList.remove('active');
        overlay.classList.remove('active');
        [clearBtn, robotBtn, laserBtn].forEach(btn => btn.disabled = false);
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
    prepareCancelBtn.addEventListener('click', closePrepareMenu);

    overlay.addEventListener('click', () => {
        if (settingsPopup.classList.contains('active')) closeSettings();
        if (preparePopup.classList.contains('active')) closePrepareMenu();
    });

    sidebarButtons.forEach((button: HTMLButtonElement) => {
        button.addEventListener('click', () => {
            const targetId: string | null = button.getAttribute('data-target');
            if (!targetId) return;
            sidebarButtons.forEach((btn) => btn.classList.remove('active'));
            settingsPanels.forEach((panel) => panel.classList.remove('active'));
            button.classList.add('active');
            const targetPanel = document.getElementById(targetId);
            if (targetPanel) targetPanel.classList.add('active');
        });
    });

    function updateDrawButtonState() {
        const btnText = clearBtn.querySelector('.btn-text')!;
        switch (drawingState) {
            case 'idle':
                clearBtn.setAttribute('data-state', 'ready');
                btnText.textContent = 'CLEAR PATH';
                clearBtn.disabled = true;
                executeBtn.disabled = true;
                prepareBtn.disabled = true;
                break;
            case 'complete':
                clearBtn.setAttribute('data-state', 'clear');
                btnText.textContent = 'CLEAR PATH';
                clearBtn.disabled = false;
                executeBtn.disabled = false;
                prepareBtn.disabled = false;
                break;
        }
    }

    function getLocalLaserState(): boolean {
        return laserBtn.classList.contains('active');
    }

    function syncUiToState(state: Partial<WebSocketMessage>) {
        if (state.isLaserOn !== undefined) {
            if (laserConfirmationTimeout) {
                clearTimeout(laserConfirmationTimeout);
                laserConfirmationTimeout = null;
            }
            const newLaserState = !!state.isLaserOn;
            laserBtn.classList.toggle('active', newLaserState);
            laserBtn.style.pointerEvents = 'auto';
        }
    }

    // --- 4. Helper: Coordinate Calculation ---
    const getCanvasCoordinates = (clientX: number, clientY: number) => {
        const rect = canvas.getBoundingClientRect();
        const scaleX = canvas.width / rect.width;
        const scaleY = canvas.height / rect.height;

        return {
            x: (clientX - rect.left) * scaleX,
            y: (clientY - rect.top) * scaleY
        };
    };

    // --- 5. WebSocket & Video Loop ---

    wsHandler.onStateUpdate = (newState: WebSocketMessage) => {
        syncUiToState(newState);
    };
    wsHandler.connect();

    const updateCanvas = (now: DOMHighResTimeStamp, metadata: VideoFrameCallbackMetadata) => {
        // 1. Draw Video Frame
        ctx.drawImage(video, 0, 0, canvas.width, canvas.height);

        // REMOVED: Real-Time Red Line Drawing (Performance Optimization)
        // We now only send coordinates, we do not visualize the path locally.

        // 2. Draw Batch Tracker Overlay (Only in Batch Mode)
        if (processingModeSwitch && !processingModeSwitch.checked && drawingTracker) {
            drawingTracker.drawOnMainCanvas();
        }

        video.requestVideoFrameCallback(updateCanvas);
    };

    const setMessage = (str: string) => {
        ctx.fillText(str, 10, 50);
    };

    video.muted = true;
    video.autoplay = true;
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
    setMessage("Loading stream");

    reader = new window.MediaMTXWebRTCReader({
        url: new URL(`http://${window.location.hostname}:8889/mystream/whep`),
        onError: (err: string) => setMessage(err),
        onTrack: (evt: RTCTrackEvent) => {
            video.srcObject = evt.streams[0];
            video.requestVideoFrameCallback(updateCanvas);

            drawingTracker = new DrawingTracker(canvas, video, `http://${window.location.hostname}:443`);

            executeBtn.disabled = true;
            prepareBtn.disabled = true;
            clearBtn.disabled = true;
        },
    });

    // --- 6. Real-Time Event Listeners ---

    const runRealTimeLoop = () => {
        // 1. Safety Check: If drawing stopped, kill the loop entirely.
        if (!isRealTimeDrawing) return;

        // 2. Capture the current position
        const currentPos = latestRealTimePos;

        // 3. Data Check: Only send if we actually have a position
        if (currentPos) {
            const vidX = (currentPos.x / canvas.width) * video.videoWidth;
            const vidY = (currentPos.y / canvas.height) * video.videoHeight;

            wsHandler.updateState({ x: vidX, y: vidY });
        }

        // 4. Loop Logic: Request the next frame regardless of whether we sent data this time
        requestAnimationFrame(runRealTimeLoop);
    }

    const handleRealTimeStart = (e: PointerEvent) => {
        if (!processingModeSwitch.checked || selectedShape !== 'freehand') return;

        e.preventDefault();
        canvas.setPointerCapture(e.pointerId);

        isRealTimeDrawing = true;


        latestRealTimePos = getCanvasCoordinates(e.clientX, e.clientY);

        // 1. Send start signal
        wsHandler.updateState({ pathEvent: 'start' });

        // 2. Start Loop
        runRealTimeLoop();
    };

    const handleRealTimeMove = (e: PointerEvent) => {
        if (!isRealTimeDrawing) return;
        e.preventDefault();

        latestRealTimePos = getCanvasCoordinates(e.clientX, e.clientY);
    };

    const handleRealTimeEnd = (e: PointerEvent) => {
        if (!isRealTimeDrawing) return;
        e.preventDefault();

        canvas.releasePointerCapture(e.pointerId);
        isRealTimeDrawing = false;
        latestRealTimePos = null;

        // End Signal
        wsHandler.updateState({ pathEvent: 'end' });
    };

    canvas.addEventListener('pointerdown', handleRealTimeStart);
    canvas.addEventListener('pointermove', handleRealTimeMove);
    canvas.addEventListener('pointerup', handleRealTimeEnd);
    canvas.addEventListener('pointercancel', handleRealTimeEnd);

    // --- 7. Controls & Logic ---

    const toggleMode = () => {
        const isRealTime = processingModeSwitch.checked;

        if (isRealTime) {
            batchUiElements.forEach(el => el.classList.add('hidden-mode'));
            if (statusControlValue) {
                statusControlValue.textContent = "REAL-TIME";
                statusControlValue.style.color = "#00ff00";
            }
            drawingTracker?.disableDrawing();
            toggleButtons.forEach(btn => btn.classList.remove('selected'));
            selectedShape = null;
        } else {
            batchUiElements.forEach(el => el.classList.remove('hidden-mode'));
            if (statusControlValue) {
                statusControlValue.textContent = "BATCH";
                statusControlValue.style.color = "";
            }
            isRealTimeDrawing = false;
            toggleButtons.forEach(btn => btn.classList.remove('selected'));
            selectedShape = null;
            drawingTracker?.disableDrawing();
        }
    };

    if (processingModeSwitch) {
        processingModeSwitch.addEventListener('change', toggleMode);
    }

    laserBtn.addEventListener('click', () => {
        laserBtn.style.pointerEvents = 'none';
        if (laserConfirmationTimeout) clearTimeout(laserConfirmationTimeout);
        changeLaserState(!getLocalLaserState());
    });

    function changeLaserState(newState: boolean) {
        const success = wsHandler.updateState({ isLaserOn: newState });
        if (success) {
            laserConfirmationTimeout = setTimeout(() => {
                console.error("No confirmation from robot. Resetting UI.");
                laserBtn.style.pointerEvents = 'auto';
            }, 2000);
        } else {
            laserBtn.style.pointerEvents = 'auto';
            console.error('Failed to send laser state update');
        }
    }

    executeBtn.addEventListener('click', async () => {
        if (!drawingTracker) return;
        executeBtn.disabled = true;
        prepareBtn.disabled = true;

        try {
            console.log('Sending coordinates to robot...');
            const result = await drawingTracker.sendCoordinates();
            if (result) console.log("Response:", result);

            drawingTracker.clearDrawing();
            drawingState = 'idle';
            updateDrawButtonState();
            closePrepareMenu();
            toggleButtons.forEach(btn => btn.classList.remove('selected'));
            selectedShape = null;
        } catch (e) {
            console.error('Error sending coordinates:', e);
            if (drawingState === 'complete') {
                executeBtn.disabled = false;
                prepareBtn.disabled = false;
            }
        }
    });

    clearBtn.addEventListener('click', () => {
        if (drawingState === 'complete' && drawingTracker) {
            drawingTracker.clearDrawing();
            drawingState = 'idle';
            updateDrawButtonState();
        }
    });

    function handleShapeSelection(button: HTMLButtonElement, shape: ShapeType) {
        const isRealTime = processingModeSwitch.checked;

        if (isRealTime && shape !== 'freehand') {
            console.log("Only Freehand/Pen is available in Real-Time mode");
            return;
        }

        const isAlreadySelected = button.classList.contains('selected');
        toggleButtons.forEach(btn => btn.classList.remove('selected'));

        if (isAlreadySelected) {
            selectedShape = null;
            drawingTracker?.disableDrawing();
        } else {
            button.classList.add('selected');
            selectedShape = shape;

            if (isRealTime) {
                drawingTracker?.clearDrawing();
            } else {
                if (drawingTracker) {
                    drawingTracker.clearDrawing();
                    drawingState = 'idle';
                    updateDrawButtonState();

                    drawingTracker.setShapeType(shape);
                    drawingTracker.enableDrawing(() => {
                        drawingState = 'complete';
                        updateDrawButtonState();
                    });
                }
            }
        }
    }

    penBtn.addEventListener('click', () => handleShapeSelection(penBtn, 'freehand'));
    squareBtn.addEventListener('click', () => handleShapeSelection(squareBtn, 'square'));
    circleBtn.addEventListener('click', () => handleShapeSelection(circleBtn, 'circle'));
    triangleBtn.addEventListener('click', () => handleShapeSelection(triangleBtn, 'triangle'));
    lineBtn.addEventListener('click', () => handleShapeSelection(lineBtn, 'line'));

    window.addEventListener('beforeunload', () => {
        if (reader !== null) reader.close();
    });

    window.addEventListener('resize', () => {
        canvas.width = window.innerWidth;
        canvas.height = window.innerHeight;
        if (drawingTracker) drawingTracker.updateCanvasSize(canvas.width, canvas.height);
    });

    // Click-to-move (Guarded against Real-Time mode)
    canvas.addEventListener('click', function (event) {
        if (processingModeSwitch.checked) return;
        if (drawingTracker?.isDrawingEnabled()) return;

        const pos = getCanvasCoordinates(event.clientX, event.clientY);

        const vidX = (pos.x / canvas.width) * video.videoWidth;
        const vidY = (pos.y / canvas.height) * video.videoHeight;

        wsHandler.updateState({ x: vidX, y: vidY });
    });

    robotBtn.addEventListener('click', () => {
        robotBtn.classList.toggle('active');
    });
});