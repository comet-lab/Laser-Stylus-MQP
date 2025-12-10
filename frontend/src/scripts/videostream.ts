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
    const speedInput = document.getElementById('speedInput') as HTMLInputElement;

    const video = document.getElementById('video') as HTMLVideoElement;
    const canvas = document.getElementById('canvas') as HTMLCanvasElement;
    const ctx = canvas.getContext("2d")!;
    const robotBtn = document.getElementById('robot-toggle-container') as HTMLButtonElement;
    const laserBtn = document.getElementById('laser-toggle-container') as HTMLButtonElement;
    const clearBtn = document.getElementById('clearBtn') as HTMLButtonElement;
    const prepareBtn = document.getElementById('prepareBtn') as HTMLButtonElement;

    const processingModeSwitch = document.getElementById('processing-mode') as HTMLInputElement;
    const thermalModeSwitch = document.getElementById('thermal-rgb-view') as HTMLInputElement;
    const transformedModeSwitch = document.getElementById('transformed-view-mode') as HTMLInputElement;
    const saveView = document.getElementById('save-view') as HTMLInputElement;
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

    const fillCheckbox = document.getElementById('fillCheckbox') as HTMLInputElement;
    const rasterPatternContainer = document.getElementById('rasterPatternContainer') as HTMLElement;
    const rasterBtnA = document.getElementById('rasterA') as HTMLButtonElement;
    const rasterBtnB = document.getElementById('rasterB') as HTMLButtonElement;


    // --- 2. State Variables ---
    let laserConfirmationTimeout: number | null = null;
    let robotConfirmationTimeout: number | null = null;
    let drawingState: 'idle' | 'complete' = 'idle';
    let selectedShape: ShapeType | null = null;
    let reader: any = null;
    let drawingTracker: DrawingTracker | null = null;
    let fillEnabled = false;
    let selectedRasterPattern: 'line_raster' | 'spiral_raster' | null = null;

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
        changeRobotState(false);
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
        preparePopup.classList.add('active');
    };

    const closePrepareMenu = (): void => {
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

    function getLocalRobotState(): boolean {
        return robotBtn.classList.contains('active');
    }

    // --- Race Condition Handling: syncUiToState ---
    // If a timeout is active (waiting for confirmation), we ONLY update
    // if the incoming message confirms our desired state.
    // If the message is "old" (e.g. says ON when we just clicked OFF), we ignore it.
    function syncUiToState(state: Partial<WebSocketMessage>) {
        if (state.isLaserOn !== undefined) {
            const incomingState = !!state.isLaserOn;
            
            if (laserConfirmationTimeout) {
                // We are waiting for a change. Only accept if it matches our optimistic UI.
                const desiredState = getLocalLaserState(); // We set this optimistically in changeLaserState
                
                if (incomingState === desiredState) {
                    clearTimeout(laserConfirmationTimeout);
                    laserConfirmationTimeout = null;
                    laserBtn.style.pointerEvents = 'auto';
                    laserBtn.classList.toggle('active', incomingState); // Ensure exact sync
                }
                // Else: Incoming state mismatch while waiting. 
                // It's likely an old packet. Ignore it to prevent flickering.
            } else {
                // No pending action, trust the server.
                laserBtn.classList.toggle('active', incomingState);
                laserBtn.style.pointerEvents = 'auto';
            }
        }

        if (state.isRobotOn !== undefined) {
            const incomingState = !!state.isRobotOn;

            if (robotConfirmationTimeout) {
                const desiredState = getLocalRobotState();

                if (incomingState === desiredState) {
                    clearTimeout(robotConfirmationTimeout);
                    robotConfirmationTimeout = null;
                    robotBtn.style.pointerEvents = 'auto';
                    robotBtn.classList.toggle('active', incomingState);
                }
            } else {
                robotBtn.classList.toggle('active', incomingState);
                robotBtn.style.pointerEvents = 'auto';
            }
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
        ctx.drawImage(video, 0, 0, canvas.width, canvas.height);

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
        if (!isRealTimeDrawing) return;
        const currentPos = latestRealTimePos;

        if (currentPos) {
            const vidX = (currentPos.x / canvas.width) * video.videoWidth;
            const vidY = (currentPos.y / canvas.height) * video.videoHeight;
            wsHandler.updateState({ x: vidX, y: vidY });
        }
        requestAnimationFrame(runRealTimeLoop);
    }

    const handleRealTimeStart = (e: PointerEvent) => {
        if (!processingModeSwitch.checked || selectedShape !== 'freehand') return;
        e.preventDefault();
        canvas.setPointerCapture(e.pointerId);
        isRealTimeDrawing = true;
        latestRealTimePos = getCanvasCoordinates(e.clientX, e.clientY);
        wsHandler.updateState({ pathEvent: 'start' });
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
        // Optimistically lock button immediately
        laserBtn.style.pointerEvents = 'none';
        if (laserConfirmationTimeout) clearTimeout(laserConfirmationTimeout);
        changeLaserState(!getLocalLaserState());
    });

    function changeLaserState(newState: boolean) {
        const updates: any = { isLaserOn: newState };

        // Optimistically update Laser UI
        laserBtn.classList.toggle('active', newState);
        
        // --- MUTUAL SHUTDOWN LOGIC ---
        // If turning off the laser, we must also turn off the robot
        if (newState === false) {
            updates.isRobotOn = false;

            // Optimistically update Robot UI
            robotBtn.classList.remove('active');
            robotBtn.style.pointerEvents = 'none';

            // Reset Robot Timeout Logic (since we are changing its state too)
            if (robotConfirmationTimeout) clearTimeout(robotConfirmationTimeout);
            robotConfirmationTimeout = setTimeout(() => {
                console.error("No confirmation from robot (triggered by laser kill). Resetting UI.");
                robotBtn.style.pointerEvents = 'auto';
            }, 2000);
        }

        const success = wsHandler.updateState(updates);

        if (success) {
            laserConfirmationTimeout = setTimeout(() => {
                console.error("No confirmation from robot (laser). Resetting UI.");
                laserBtn.style.pointerEvents = 'auto';
            }, 2000);
        } else {
            // Revert on send failure
            laserBtn.style.pointerEvents = 'auto';
            robotBtn.style.pointerEvents = 'auto';
            console.error('Failed to send laser state update');
        }
    }

    robotBtn.addEventListener('click', () => {
        // Optimistically lock button immediately
        robotBtn.style.pointerEvents = 'none';
        if (robotConfirmationTimeout) clearTimeout(robotConfirmationTimeout);
        changeRobotState(!getLocalRobotState());
    });

    function changeRobotState(newState: boolean) {
        const updates: any = { isRobotOn: newState };

        // Optimistically update Robot UI
        robotBtn.classList.toggle('active', newState);

        // --- MUTUAL SHUTDOWN LOGIC ---
        // If turning off the robot, we must also turn off the laser
        if (newState === false) {
            updates.isLaserOn = false;

            // Optimistically update Laser UI
            laserBtn.classList.remove('active');
            laserBtn.style.pointerEvents = 'none';

            // Reset Laser Timeout Logic
            if (laserConfirmationTimeout) clearTimeout(laserConfirmationTimeout);
            laserConfirmationTimeout = setTimeout(() => {
                console.error("No confirmation from robot (triggered by robot kill). Resetting UI.");
                laserBtn.style.pointerEvents = 'auto';
            }, 2000);
        }

        const success = wsHandler.updateState(updates);
        
        if (success) {
            robotConfirmationTimeout = setTimeout(() => {
                console.error("No confirmation from robot (robot). Resetting UI.");
                robotBtn.style.pointerEvents = 'auto';
            }, 2000);
        } else {
            // Revert on send failure
            robotBtn.style.pointerEvents = 'auto';
            laserBtn.style.pointerEvents = 'auto';
            console.error('Failed to send robot state update');
        }
    }

    function clearDrawing() {
        if (!drawingTracker) { return }
        drawingTracker.clearDrawing();
        drawingState = 'idle';
        updateDrawButtonState();
    }

    function cancelDrawing() {
        if (!drawingTracker) { return }
        clearDrawing();
        drawingTracker.disableDrawing();
        selectedShape = null;
        toggleButtons.forEach(btn => btn.classList.remove('selected'));
    }

    executeBtn.addEventListener('click', async () => {
        if (!drawingTracker) return;
        executeBtn.disabled = true;
        prepareBtn.disabled = true;

        const speed = parseFloat(speedInput.value);

        if (isNaN(speed) || speed <= 0) {
            alert("Please enter a valid speed greater than 0 m/s");
            executeBtn.disabled = false;
            prepareBtn.disabled = false;
            return;
        }

        try {
            console.log(`Executing path at speed: ${speed / 1000} m/s`);
            const result = await drawingTracker.executePath(speed, String(selectedRasterPattern));

            if (result) {
                console.log("Execution started successfully");
                console.log("Response:", result);
            }
            cancelDrawing();
            closePrepareMenu();
        } catch (e) {
            console.error('Error sending coordinates:', e);
            if (drawingState === 'complete') {
                executeBtn.disabled = false;
                prepareBtn.disabled = false;
            }
        }
    });

    saveView.addEventListener('click', async () => {
        if (!drawingTracker) return;
        const transformedView = transformedModeSwitch.checked;
        const thermalView = thermalModeSwitch.checked;
        const result = await drawingTracker.updateViewSettings(transformedView, thermalView);

        if (result) {
            console.log("Updated the view settings successfully");
            console.log("Response:", result);
        }
    });

    clearBtn.addEventListener('click', () => {
        if (drawingState === 'complete' && drawingTracker) {
            clearDrawing();
        }
    });

    fillCheckbox.addEventListener('change', () => {
        fillEnabled = fillCheckbox.checked;

        if (fillEnabled) {
            rasterPatternContainer.classList.remove('hidden');
        } else {
            rasterPatternContainer.classList.add('hidden');
            selectedRasterPattern = null;
            rasterBtnA.classList.remove('active');
            rasterBtnB.classList.remove('active');
        }
    });

    function selectRaster(btn: HTMLButtonElement, pattern: 'line_raster' | 'spiral_raster') {
        rasterBtnA.classList.remove('active');
        rasterBtnB.classList.remove('active');
        btn.classList.add('active');
        selectedRasterPattern = pattern;
        console.log("Raster pattern selected:", pattern);
    }

    rasterBtnA.addEventListener('click', () => selectRaster(rasterBtnA, 'line_raster'));
    rasterBtnB.addEventListener('click', () => selectRaster(rasterBtnB, 'spiral_raster'));

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
                    clearDrawing();
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

    const handleResize = () => {
        canvas.width = canvas.offsetWidth;
        canvas.height = canvas.offsetHeight;
        if (drawingTracker) {
            drawingTracker.updateCanvasSize(canvas.width, canvas.height);
        }
        if (drawingTracker?.isDrawingEnabled()) {
            clearDrawing();
        }
    };

    window.addEventListener('resize', handleResize);
    handleResize();
});