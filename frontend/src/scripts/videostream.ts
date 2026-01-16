import { DrawingTracker } from './drawing/DrawingTracker';
import { ShapeType } from './drawing/types';
import { WebSocketHandler, WebSocketMessage } from './classes/WebSocketHandler';

declare global {
    interface Window {
        MediaMTXWebRTCReader: any;
    }
}

window.addEventListener('load', () => {
    // --- 1. Get DOM Elements ---
    const viewport = document.getElementById('viewport') as HTMLElement; // NEW: Parent container
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
    
    // Shape Buttons
    const penBtn = document.getElementById('penBtn') as HTMLButtonElement;
    const squareBtn = document.getElementById('squareBtn') as HTMLButtonElement;
    const circleBtn = document.getElementById('circleBtn') as HTMLButtonElement;
    const triangleBtn = document.getElementById('triangleBtn') as HTMLButtonElement;
    const lineBtn = document.getElementById('lineBtn') as HTMLButtonElement;
    
    const toggleButtons: NodeListOf<HTMLButtonElement> = document.querySelectorAll('#middle-icon-section .icon-btn');
    const sidebarButtons: NodeListOf<HTMLButtonElement> = document.querySelectorAll('.settings-sidebar .sidebar-btn');
    const settingsPanels: NodeListOf<HTMLElement> = document.querySelectorAll('.settings-main .settings-panel');
    const fillCheckbox = document.getElementById('fillCheckbox') as HTMLInputElement;
    const rasterPatternContainer = document.getElementById('rasterPatternContainer') as HTMLElement;
    const rasterBtnA = document.getElementById('rasterA') as HTMLButtonElement;
    const rasterBtnB = document.getElementById('rasterB') as HTMLButtonElement;

    // --- 2. State Variables ---
    let laserConfirmationTimeout: number | null = null;
    let robotConfirmationTimeout: number | null = null;
    let selectedShape: ShapeType | null = null; 
    let drawnShapeType: ShapeType | null = null; 
    let reader: any = null;
    let drawingTracker: DrawingTracker | null = null;
    let fillEnabled = false;
    let selectedRasterPattern: 'line_raster' | 'spiral_raster' | null = null;
    let isRealTimeDrawing = false;
    let latestRealTimePos: { x: number, y: number } | null = null;
    const wsHandler = new WebSocketHandler(null);

    // --- 3. UI Helpers & State Machine ---
    
    function updateDrawButtonState() {
        const hasShape = drawnShapeType !== null;

        // 1. Action Buttons
        clearBtn.disabled = !hasShape;
        prepareBtn.disabled = !hasShape;
        executeBtn.disabled = !hasShape;

        // 2. Shape Tool Buttons
        if (hasShape) {
            penBtn.disabled      = (drawnShapeType !== 'freehand');
            squareBtn.disabled   = (drawnShapeType !== 'square');
            circleBtn.disabled   = (drawnShapeType !== 'circle');
            triangleBtn.disabled = (drawnShapeType !== 'triangle');
            lineBtn.disabled     = (drawnShapeType !== 'line');
        } else {
            toggleButtons.forEach(btn => btn.disabled = false);
        }
    }

    const openSettings = (): void => {
        if (drawingTracker && drawingTracker.hasShape()) {
            drawingTracker.clearDrawing();
            selectedShape = null;
            drawnShapeType = null;
            toggleButtons.forEach(btn => btn.classList.remove('selected'));
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
    prepareBtn.addEventListener('click', () => preparePopup.classList.add('active'));
    prepareCloseBtn.addEventListener('click', () => preparePopup.classList.remove('active'));
    prepareCancelBtn.addEventListener('click', () => preparePopup.classList.remove('active'));

    overlay.addEventListener('click', () => {
        if (settingsPopup.classList.contains('active')) closeSettings();
        if (preparePopup.classList.contains('active')) preparePopup.classList.remove('active');
    });

    sidebarButtons.forEach((button: HTMLButtonElement) => {
        button.addEventListener('click', () => {
            const targetId = button.getAttribute('data-target');
            if (!targetId) return;
            sidebarButtons.forEach((btn) => btn.classList.remove('active'));
            settingsPanels.forEach((panel) => panel.classList.remove('active'));
            button.classList.add('active');
            const targetPanel = document.getElementById(targetId);
            if (targetPanel) targetPanel.classList.add('active');
        });
    });

    // --- 4. WebSocket & State Sync ---
    function getLocalLaserState(): boolean { return laserBtn.classList.contains('active'); }
    function getLocalRobotState(): boolean { return robotBtn.classList.contains('active'); }

    function syncUiToState(state: Partial<WebSocketMessage>) {
        if (state.isLaserOn !== undefined) {
            const incomingState = !!state.isLaserOn;
            if (laserConfirmationTimeout) {
                if (incomingState !== getLocalLaserState()) {
                    clearTimeout(laserConfirmationTimeout);
                    laserConfirmationTimeout = null;
                    laserBtn.classList.toggle('active', incomingState);
                    laserBtn.style.pointerEvents = 'auto';
                }
            } else {
                laserBtn.classList.toggle('active', incomingState);
                laserBtn.style.pointerEvents = 'auto';
            }
        }
        if (state.isRobotOn !== undefined) {
            const incomingState = !!state.isRobotOn;
            if (robotConfirmationTimeout) {
                if (incomingState !== getLocalRobotState()) {
                    clearTimeout(robotConfirmationTimeout);
                    robotConfirmationTimeout = null;
                    robotBtn.classList.toggle('active', incomingState);
                    robotBtn.style.pointerEvents = 'auto';
                }
            } else {
                robotBtn.classList.toggle('active', incomingState);
                robotBtn.style.pointerEvents = 'auto';
            }
        }
    }

    wsHandler.onStateUpdate = (newState: WebSocketMessage) => syncUiToState(newState);
    wsHandler.connect();

    // --- 5. Canvas Loop ---
    const updateCanvas = () => {
        ctx.drawImage(video, 0, 0, canvas.width, canvas.height);

        if (processingModeSwitch && !processingModeSwitch.checked && drawingTracker) {
            drawingTracker.render();
        }

        video.requestVideoFrameCallback(updateCanvas);
    };

    const setMessage = (str: string) => { ctx.fillText(str, 10, 50); };

    video.muted = true;
    video.autoplay = true;
    // Set initial size based on Viewport, not window innerWidth directly
    canvas.width = viewport.offsetWidth;
    canvas.height = viewport.offsetHeight;
    setMessage("Loading stream");

    reader = new window.MediaMTXWebRTCReader({
        url: new URL(`http://${window.location.hostname}:8889/mystream/whep`),
        onError: (err: string) => setMessage(err),
        onTrack: (evt: RTCTrackEvent) => {
            if (evt.track.kind === 'video') {
                video.srcObject = evt.streams[0];
                video.requestVideoFrameCallback(updateCanvas);
                
                if (drawingTracker) {
                    drawingTracker.dispose(); 
                }

                drawingTracker = new DrawingTracker(
                    canvas, 
                    video, 
                    `http://${window.location.hostname}:443`,
                    () => {
                        if (selectedShape) {
                            drawnShapeType = selectedShape;
                            updateDrawButtonState();
                        }
                    }
                );
                
                updateDrawButtonState();
            }
        },
    });

    // --- 6. Real-Time Logic ---
    const getCanvasCoordinates = (clientX: number, clientY: number) => {
        const rect = canvas.getBoundingClientRect();
        return { x: clientX - rect.left, y: clientY - rect.top };
    };

    const runRealTimeLoop = () => {
        if (!isRealTimeDrawing) return;
        if (latestRealTimePos) {
            const vidX = (latestRealTimePos.x / canvas.width) * video.videoWidth;
            const vidY = (latestRealTimePos.y / canvas.height) * video.videoHeight;
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

    canvas.addEventListener('pointerdown', handleRealTimeStart);
    canvas.addEventListener('pointermove', (e) => {
        if (isRealTimeDrawing) {
            e.preventDefault();
            latestRealTimePos = getCanvasCoordinates(e.clientX, e.clientY);
        }
    });
    const handleRealTimeEnd = (e: PointerEvent) => {
        if (!isRealTimeDrawing) return;
        e.preventDefault();
        canvas.releasePointerCapture(e.pointerId);
        isRealTimeDrawing = false;
        latestRealTimePos = null;
        wsHandler.updateState({ pathEvent: 'end' });
    };
    canvas.addEventListener('pointerup', handleRealTimeEnd);
    canvas.addEventListener('pointercancel', handleRealTimeEnd);

    // --- 7. Controls ---

    const toggleMode = () => {
        const isRealTime = processingModeSwitch.checked;
        if (isRealTime) {
            batchUiElements.forEach(el => el.classList.add('hidden-mode'));
            statusControlValue.textContent = "REAL-TIME";
            statusControlValue.style.color = "#00ff00";
            drawingTracker?.disableDrawing();
        } else {
            batchUiElements.forEach(el => el.classList.remove('hidden-mode'));
            statusControlValue.textContent = "BATCH";
            statusControlValue.style.color = "";
            drawingTracker?.disableDrawing();
        }
        toggleButtons.forEach(btn => {
            btn.classList.remove('selected');
            btn.disabled = false;
        });
        selectedShape = null;
        drawnShapeType = null;
        updateDrawButtonState();
    };
    processingModeSwitch.addEventListener('change', toggleMode);

    // Laser/Robot Toggle Logic
    laserBtn.addEventListener('click', () => {
        laserBtn.style.pointerEvents = 'none';
        if (laserConfirmationTimeout) clearTimeout(laserConfirmationTimeout);
        changeLaserState(!getLocalLaserState());
    });
    robotBtn.addEventListener('click', () => {
        robotBtn.style.pointerEvents = 'none';
        if (robotConfirmationTimeout) clearTimeout(robotConfirmationTimeout);
        changeRobotState(!getLocalRobotState());
    });

    function changeLaserState(newState: boolean) {
        laserBtn.style.pointerEvents = 'none';
        const updates: any = { isLaserOn: newState };
        if (newState === false && getLocalRobotState() === true) {
            updates.isRobotOn = false;
            robotBtn.style.pointerEvents = 'none';
            if (robotConfirmationTimeout) clearTimeout(robotConfirmationTimeout);
            robotConfirmationTimeout = setTimeout(() => { robotBtn.style.pointerEvents = 'auto'; }, 2000);
        }
        wsHandler.updateState(updates);
        if (laserConfirmationTimeout) clearTimeout(laserConfirmationTimeout);
        laserConfirmationTimeout = setTimeout(() => { laserBtn.style.pointerEvents = 'auto'; }, 2000);
    }

    function changeRobotState(newState: boolean) {
        robotBtn.style.pointerEvents = 'none';
        const updates: any = { isRobotOn: newState };
        if (newState === false && getLocalLaserState() === true) {
            updates.isLaserOn = false;
            laserBtn.style.pointerEvents = 'none';
            if (laserConfirmationTimeout) clearTimeout(laserConfirmationTimeout);
            laserConfirmationTimeout = setTimeout(() => { laserBtn.style.pointerEvents = 'auto'; }, 2000);
        }
        wsHandler.updateState(updates);
        if (robotConfirmationTimeout) clearTimeout(robotConfirmationTimeout);
        robotConfirmationTimeout = setTimeout(() => { robotBtn.style.pointerEvents = 'auto'; }, 2000);
    }

    // --- Shape Selection ---
    function handleShapeSelection(button: HTMLButtonElement, shape: ShapeType) {
        if (processingModeSwitch.checked && shape !== 'freehand') return;
        if (button.disabled) return;

        const isAlreadySelected = button.classList.contains('selected');
        
        if (isAlreadySelected) {
            button.classList.remove('selected');
            selectedShape = null;
            drawingTracker?.disableDrawing();
        } else {
            toggleButtons.forEach(btn => btn.classList.remove('selected'));
            button.classList.add('selected');
            selectedShape = shape;

            if (processingModeSwitch.checked) {
                drawingTracker?.clearDrawing();
                drawnShapeType = null;
            } else {
                if (drawingTracker) {
                    drawingTracker.setShapeType(shape);
                    drawingTracker.enableDrawing();
                }
            }
        }
        updateDrawButtonState();
    }

    penBtn.addEventListener('click', () => handleShapeSelection(penBtn, 'freehand'));
    squareBtn.addEventListener('click', () => handleShapeSelection(squareBtn, 'square'));
    circleBtn.addEventListener('click', () => handleShapeSelection(circleBtn, 'circle'));
    triangleBtn.addEventListener('click', () => handleShapeSelection(triangleBtn, 'triangle'));
    lineBtn.addEventListener('click', () => handleShapeSelection(lineBtn, 'line'));

    canvas.addEventListener('mouseup', () => { setTimeout(updateDrawButtonState, 50); });
    canvas.addEventListener('touchend', () => { setTimeout(updateDrawButtonState, 50); });

    executeBtn.addEventListener('click', async () => {
        if (!drawingTracker) return;
        const speed = parseFloat(speedInput.value);
        if (isNaN(speed) || speed <= 0) {
            alert("Invalid speed");
            return;
        }

        executeBtn.disabled = true;
        prepareBtn.disabled = true;

        try {
            await drawingTracker.executePath(speed, String(selectedRasterPattern));
            
            drawingTracker.clearDrawing();
            toggleButtons.forEach(btn => {
                btn.classList.remove('selected');
                btn.disabled = false;
            });
            selectedShape = null;
            drawnShapeType = null;
            updateDrawButtonState();
            preparePopup.classList.remove('active');
        } catch (e) {
            console.error(e);
            executeBtn.disabled = false;
            prepareBtn.disabled = false;
        }
    });

    clearBtn.addEventListener('click', () => {
        drawingTracker?.clearDrawing();
        drawnShapeType = null;
        updateDrawButtonState();
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

    rasterBtnA.addEventListener('click', () => {
        rasterBtnA.classList.add('active');
        rasterBtnB.classList.remove('active');
        selectedRasterPattern = 'line_raster';
    });
    rasterBtnB.addEventListener('click', () => {
        rasterBtnB.classList.add('active');
        rasterBtnA.classList.remove('active');
        selectedRasterPattern = 'spiral_raster';
    });

    saveView.addEventListener('click', async () => {
        if (!drawingTracker) return;
        await drawingTracker.updateViewSettings(transformedModeSwitch.checked, thermalModeSwitch.checked);
    });

    // --- FIX: Proper Resize Logic ---
    window.addEventListener('resize', () => {
        // Use Viewport as source of truth. DO NOT manually set canvas.width here.
        // Let Fabric handle the sync via the tracker.
        if (drawingTracker) {
            drawingTracker.updateCanvasSize(viewport.offsetWidth, viewport.offsetHeight);
        }
    });
});