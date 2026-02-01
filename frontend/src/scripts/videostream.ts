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
    const viewport = document.getElementById('viewport') as HTMLElement;
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
    const transformedModeSwitch = document.getElementById('transformed-view-mode') as HTMLInputElement;
    const saveView = document.getElementById('save-view') as HTMLInputElement;
    const batchUiElements = document.querySelectorAll('.batch-ui');
    const robotMarker = document.getElementById('robot-marker') as HTMLElement;

    // Shape Buttons
    const penBtn = document.getElementById('penBtn') as HTMLButtonElement;
    const squareBtn = document.getElementById('squareBtn') as HTMLButtonElement;
    const circleBtn = document.getElementById('circleBtn') as HTMLButtonElement;
    const triangleBtn = document.getElementById('triangleBtn') as HTMLButtonElement;
    const lineBtn = document.getElementById('lineBtn') as HTMLButtonElement;
    const markerBtn = document.getElementById('markerBtn') as HTMLButtonElement;
    const clearMarkersBtn = document.getElementById('clearMarkersBtn') as HTMLButtonElement;

    // Settings & Panels
    const toggleButtons: NodeListOf<HTMLButtonElement> = document.querySelectorAll('#middle-icon-section .icon-btn');
    const sidebarButtons: NodeListOf<HTMLButtonElement> = document.querySelectorAll('.settings-sidebar .sidebar-btn');
    const settingsPanels: NodeListOf<HTMLElement> = document.querySelectorAll('.settings-main .settings-panel');

    // Fill & Accordion UI
    const fillAccordionToggle = document.getElementById('fillAccordionToggle') as HTMLButtonElement;
    const fillSettingsPanel = document.getElementById('fillSettingsPanel') as HTMLElement;
    const rasterBtnA = document.getElementById('rasterA') as HTMLButtonElement;
    const rasterBtnB = document.getElementById('rasterB') as HTMLButtonElement;
    const rasterDensityInput = document.getElementById('densityRaster') as HTMLInputElement;

    // Fixtures Mode Elements
    const fixturesTools = document.getElementById('fixtures-tools') as HTMLElement;
    const drawingTools = document.getElementById('drawing-tools') as HTMLElement;
    const thermalTools = document.getElementById('thermal-tools') as HTMLElement;
    const roundBrushBtn = document.getElementById('roundBrushBtn') as HTMLButtonElement;
    const squareBrushBtn = document.getElementById('squareBrushBtn') as HTMLButtonElement;
    const brushSizeSlider = document.getElementById('brushSizeSlider') as HTMLInputElement;
    const clearBoundaryBtn = document.getElementById('clearBoundaryBtn') as HTMLButtonElement;
    const applyFixturesBtn = document.getElementById('applyFixturesBtn') as HTMLButtonElement;
    const fixturesUiElements = document.querySelectorAll('.fixtures-ui-only');
    const drawingUiElements = document.querySelectorAll('.drawing-ui-only');
    const thermalUiElements = document.querySelectorAll('.thermal-ui');
    const eraserBrushBtn = document.getElementById('eraserBtn') as HTMLButtonElement;

    // heat display stuff
    const averageHeatDisplay = document.getElementById('average-heat-display') as HTMLElement;

    // --- 2. State Variables ---
    let laserConfirmationTimeout: number | null = null;
    let robotConfirmationTimeout: number | null = null;
    let selectedShape: ShapeType | 'marker' | null = null;
    let drawnShapeType: ShapeType | null = null;
    let reader: any = null;
    let drawingTracker: DrawingTracker | null = null;
    let isRealTimeDrawing = false;
    let latestRealTimePos: { x: number, y: number } | null = null;
    const wsHandler = new WebSocketHandler(null);

    // Fill State
    let fillEnabled = false;
    let selectedRasterPattern: 'line_raster' | 'spiral_raster' | null = 'line_raster';

    // Fixtures Mode State
    type CurrentMode = 'drawing' | 'thermal' | 'fixtures';
    let currentMode: CurrentMode = 'drawing';
    let selectedBrushType: 'round' | 'square' | null = null;
    let isEraserActive = false;

    // --- 3. UI Helper Functions ---
    function updateDrawButtonState() {
        const hasShape = drawnShapeType !== null;

        // Action Buttons
        clearBtn.disabled = !hasShape;
        prepareBtn.disabled = !hasShape;
        executeBtn.disabled = !hasShape;

        // Shape Tool Buttons
        if (hasShape) {
            penBtn.disabled = (drawnShapeType !== 'freehand');
            squareBtn.disabled = (drawnShapeType !== 'square');
            circleBtn.disabled = (drawnShapeType !== 'circle');
            triangleBtn.disabled = (drawnShapeType !== 'triangle');
            lineBtn.disabled = (drawnShapeType !== 'line');
            markerBtn.disabled = true;
        } else {
            toggleButtons.forEach(btn => btn.disabled = false);
        }
    }

    function updateFixturesButtonState() {
        const hasFixtures = drawingTracker?.hasFixtures() ?? false;
        const canApply = drawingTracker?.canApplyFixtures() ?? false;

        clearBoundaryBtn.disabled = !hasFixtures;
        applyFixturesBtn.disabled = !canApply;

        if (hasFixtures) {
            roundBrushBtn.disabled = false;
            squareBrushBtn.disabled = false;
            eraserBrushBtn.disabled = false;
        } else {
            roundBrushBtn.disabled = false;
            squareBrushBtn.disabled = false;
            eraserBrushBtn.disabled = true;
        }
    }

    function updateThermalButtonState() {
        if (!drawingTracker) return;
        const hasMarkers = drawingTracker.hasMarkers();
        clearMarkersBtn.disabled = !hasMarkers;
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

    // --- 4. Event Listeners ---

    // Modal Interactions
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

    // --- 5. WebSocket & State Sync ---
    function getLocalLaserState(): boolean { return laserBtn.classList.contains('active'); }
    function getLocalRobotState(): boolean { return robotBtn.classList.contains('active'); }

    function syncUiToState(state: Partial<WebSocketMessage>) {
        // Coordinate/Marker Sync
        if (state.x !== undefined && state.y !== undefined) {
            const containerWidth = viewport.offsetWidth;
            const containerHeight = viewport.offsetHeight;
            const videoWidth = video.videoWidth;
            const videoHeight = video.videoHeight;

            if (videoWidth > 0 && videoHeight > 0) {
                const cssLeft = (state.x / videoWidth) * containerWidth;
                const cssTop = (state.y / videoHeight) * containerHeight;
                robotMarker.style.left = `${cssLeft}px`;
                robotMarker.style.top = `${cssTop}px`;
                robotMarker.style.display = 'block';
            }
        }

        if (state.averageHeat !== undefined) {
            updateAverageHeat(state.averageHeat);
        }

        if (state.heat_markers && drawingTracker) {
            drawingTracker.updateMarkerTemperatures(state.heat_markers);
        }

        if (state.isLaserOn !== undefined) {
            syncToggleState(laserBtn, !!state.isLaserOn, laserConfirmationTimeout);
        }
        if (state.isRobotOn !== undefined) {
            syncToggleState(robotBtn, !!state.isRobotOn, robotConfirmationTimeout);
        }
    }

    function syncToggleState(
        btn: HTMLElement,
        incomingState: boolean,
        timeoutRef: number | null
    ) {
        if (timeoutRef) {
            if (incomingState === btn.classList.contains('active')) {
                btn.style.pointerEvents = 'auto';
            }
        } else {
            btn.classList.toggle('active', incomingState);
            btn.style.pointerEvents = 'auto';
        }
    }

    wsHandler.onStateUpdate = (newState: WebSocketMessage) => syncUiToState(newState);
    wsHandler.connect();

    // --- 6. Canvas Loop ---
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
                        if (selectedShape && selectedShape !== 'marker') {
                            drawnShapeType = selectedShape;
                            updateDrawButtonState();
                        }
                    },
                    () => {
                        updateFixturesButtonState();
                    },
                    () => {
                        // On markers change
                        updateThermalButtonState();
                    }
                );

                updateDrawButtonState();
                updateFixturesButtonState();
                updateThermalButtonState();
            }
        },
    });

    // --- 7. Real-Time Logic ---
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

    viewport.addEventListener('pointerdown', handleRealTimeStart);
    viewport.addEventListener('pointermove', (e) => {
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
    viewport.addEventListener('pointerup', handleRealTimeEnd);
    viewport.addEventListener('pointercancel', handleRealTimeEnd);

    // --- 8. Controls ---

    const toggleMode = () => {
        const isRealTime = processingModeSwitch.checked;
        if (isRealTime) {
            batchUiElements.forEach(el => el.classList.add('hidden-mode'));
            drawingTracker?.disableDrawing();
        } else {
            batchUiElements.forEach(el => el.classList.remove('hidden-mode'));
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

    const modeButtons = document.querySelectorAll('.mode-btn');

    modeButtons.forEach(btn => {
        btn.addEventListener('click', () => {
            modeButtons.forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
            const mode = btn.id;
            switchMode(mode);
        });
    });

    document.getElementById('drawingBtn')?.classList.add('active');

    function switchMode(modeId: string) {
        console.log("Switching to mode:", modeId);

        if (currentMode === 'fixtures' && modeId !== 'fixturesBtn') {
            if (drawingTracker && drawingTracker.canApplyFixtures()) {
                drawingTracker.clearFixtures();
                selectedBrushType = null;
                isEraserActive = false;
                roundBrushBtn.classList.remove('selected');
                squareBrushBtn.classList.remove('selected');
                eraserBrushBtn.classList.remove('selected');
            }
        }

        // Reset Marker Mode when switching main tabs
        if (currentMode === 'thermal' && modeId !== 'thermalBtn') {
            drawingTracker?.disableMarkerMode();
            markerBtn.classList.remove('selected');
        }

        // Logic to Hide/Show markers based on Mode
        if (modeId === 'thermalBtn') {
            drawingTracker?.showMarkers();
            updateThermalButtonState();
        }

        if (modeId === 'fixturesBtn') {
            currentMode = 'fixtures';
            drawingTools.classList.add('hidden');
            thermalTools.classList.add('hidden');
            fixturesTools.classList.remove('hidden');

            thermalUiElements.forEach(el => el.classList.add('hidden'));
            drawingUiElements.forEach(el => el.classList.add('hidden'));
            fixturesUiElements.forEach(el => el.classList.remove('hidden'));

            drawingTracker?.disableDrawing();
            drawingTracker?.enableFixturesMode();
            updateFixturesButtonState();

        } else if (modeId === 'thermalBtn') {
            currentMode = 'thermal';
            drawingTools.classList.add('hidden');
            thermalTools.classList.remove('hidden');
            fixturesTools.classList.add('hidden');

            drawingUiElements.forEach(el => el.classList.add('hidden'));
            fixturesUiElements.forEach(el => el.classList.add('hidden'));

            drawingTracker?.disableFixturesMode();
            drawingTracker?.disableDrawing();
            if (drawingTracker?.hasFixtures()) {
                drawingTracker?.showFixtures();
            }
            thermalUiElements.forEach(el => el.classList.remove('hidden'));

            // Re-apply selection state if we came back to this mode
            if (selectedShape === 'marker') {
                markerBtn.classList.add('selected');
                drawingTracker?.enableMarkerMode();
            }

        } else { // drawing mode
            currentMode = 'drawing';
            drawingTools.classList.remove('hidden');
            thermalTools.classList.add('hidden');
            fixturesTools.classList.add('hidden');

            drawingUiElements.forEach(el => el.classList.remove('hidden'));
            fixturesUiElements.forEach(el => el.classList.add('hidden'));
            thermalUiElements.forEach(el => el.classList.add('hidden'));

            drawingTracker?.disableFixturesMode();

            if (drawingTracker?.hasFixtures()) {
                drawingTracker?.showFixtures();
            }

            if (drawnShapeType) {
                drawingTracker?.disableDrawing();
                toggleButtons.forEach(btn => btn.classList.remove('selected'));
                if (drawnShapeType === 'freehand') penBtn.classList.add('selected');
                else if (drawnShapeType === 'square') squareBtn.classList.add('selected');
                else if (drawnShapeType === 'circle') circleBtn.classList.add('selected');
                else if (drawnShapeType === 'triangle') triangleBtn.classList.add('selected');
                else if (drawnShapeType === 'line') lineBtn.classList.add('selected');

            } else if (selectedShape && selectedShape !== 'marker') {
                drawingTracker?.setShapeType(selectedShape);
                drawingTracker?.enableDrawing();
                toggleButtons.forEach(btn => btn.classList.remove('selected'));
                if (selectedShape === 'freehand') penBtn.classList.add('selected');
                else if (selectedShape === 'square') squareBtn.classList.add('selected');
                else if (selectedShape === 'circle') circleBtn.classList.add('selected');
                else if (selectedShape === 'triangle') triangleBtn.classList.add('selected');
                else if (selectedShape === 'line') lineBtn.classList.add('selected');
            } else {
                drawingTracker?.disableDrawing();
                toggleButtons.forEach(btn => btn.classList.remove('selected'));
            }
            updateDrawButtonState();
        }
    }

    function handleBrushSelection(brushType: 'round' | 'square') {
        if (roundBrushBtn.disabled && brushType === 'round') return;
        if (squareBrushBtn.disabled && brushType === 'square') return;

        const isAlreadySelected = selectedBrushType === brushType;

        if (isAlreadySelected) {
            if (brushType === 'round') roundBrushBtn.classList.remove('selected');
            else squareBrushBtn.classList.remove('selected');
            selectedBrushType = null;
            isEraserActive = false;
            drawingTracker?.disableFixturesBrush();
        } else {
            roundBrushBtn.classList.remove('selected');
            squareBrushBtn.classList.remove('selected');
            eraserBrushBtn.classList.remove('selected');

            if (brushType === 'round') roundBrushBtn.classList.add('selected');
            else squareBrushBtn.classList.add('selected');

            selectedBrushType = brushType;
            isEraserActive = false;

            const brushSize = parseInt(brushSizeSlider.value);
            drawingTracker?.setFixturesBrush(brushType, brushSize, false);
        }
        updateFixturesButtonState();
    }

    function handleEraserSelection() {
        if (eraserBrushBtn.disabled) return;
        const isAlreadySelected = isEraserActive;

        if (isAlreadySelected) {
            eraserBrushBtn.classList.remove('selected');
            selectedBrushType = null;
            isEraserActive = false;
            drawingTracker?.disableFixturesBrush();
        } else {
            roundBrushBtn.classList.remove('selected');
            squareBrushBtn.classList.remove('selected');
            eraserBrushBtn.classList.add('selected');

            selectedBrushType = 'round'; // Eraser uses round shape
            isEraserActive = true;

            const brushSize = parseInt(brushSizeSlider.value);
            drawingTracker?.setFixturesBrush('round', brushSize, true);
        }
        updateFixturesButtonState();
    }

    roundBrushBtn.addEventListener('click', () => handleBrushSelection('round'));
    squareBrushBtn.addEventListener('click', () => handleBrushSelection('square'));
    eraserBrushBtn.addEventListener('click', () => handleEraserSelection());

    brushSizeSlider.addEventListener('input', () => {
        if (selectedBrushType && drawingTracker) {
            const brushSize = parseInt(brushSizeSlider.value);
            drawingTracker.setFixturesBrush(selectedBrushType, brushSize, isEraserActive);
        }
    });

    clearBoundaryBtn.addEventListener('click', async () => {
        if (!drawingTracker) return;
        clearBoundaryBtn.disabled = true;

        try {
            drawingTracker.clearFixtures();
            await drawingTracker.clearFixturesOnServer();
        } catch (e) {
            console.error(e);
        }

        drawingTracker.disableFixturesBrush();
        selectedBrushType = null;
        isEraserActive = false;
        roundBrushBtn.classList.remove('selected');
        squareBrushBtn.classList.remove('selected');
        eraserBrushBtn.classList.remove('selected');
        updateFixturesButtonState();
    });

    applyFixturesBtn.addEventListener('click', async () => {
        if (!drawingTracker) return;
        applyFixturesBtn.disabled = true;
        clearBoundaryBtn.disabled = true;

        try {
            await drawingTracker.executeFixtures();
            drawingTracker.disableFixturesBrush();
            selectedBrushType = null;
            isEraserActive = false;
            roundBrushBtn.classList.remove('selected');
            squareBrushBtn.classList.remove('selected');
            eraserBrushBtn.classList.remove('selected');
            updateFixturesButtonState();
        } catch (e) {
            console.error(e);
            applyFixturesBtn.disabled = false;
            clearBoundaryBtn.disabled = false;
        }
    });

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

    function handleShapeSelection(button: HTMLButtonElement, shape: ShapeType | 'marker') {
        if (processingModeSwitch.checked && shape !== 'freehand') return;
        if (button.disabled) return;

        const isAlreadySelected = button.classList.contains('selected');

        if (isAlreadySelected) {
            button.classList.remove('selected');
            selectedShape = null;
            drawingTracker?.disableDrawing();
            drawingTracker?.disableMarkerMode();
        } else {
            toggleButtons.forEach(btn => btn.classList.remove('selected'));
            button.classList.add('selected');
            selectedShape = shape;

            if (processingModeSwitch.checked) {
                drawingTracker?.clearDrawing();
                drawnShapeType = null;
            } else {
                if (drawingTracker) {
                    if (shape === 'marker') {
                        drawingTracker.enableMarkerMode();
                    } else {
                        drawingTracker.setShapeType(shape);
                        drawingTracker.enableDrawing();
                    }
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
    markerBtn.addEventListener('click', () => handleShapeSelection(markerBtn, 'marker'));

    canvas.addEventListener('mouseup', () => { setTimeout(updateDrawButtonState, 50); });
    canvas.addEventListener('touchend', () => { setTimeout(updateDrawButtonState, 50); });

    executeBtn.addEventListener('click', async () => {
        if (!drawingTracker) return;

        const speed = parseFloat(speedInput.value);
        let density = 0;
        if (fillEnabled) {
            density = parseFloat(rasterDensityInput.value);
        }
        if (isNaN(speed) || speed <= 0) {
            alert("Invalid speed");
            return;
        }

        executeBtn.disabled = true;
        prepareBtn.disabled = true;

        try {
            await drawingTracker.executePath(speed, String(selectedRasterPattern), density, fillEnabled);
            drawingTracker.clearDrawing();
            drawnShapeType = null;
            toggleButtons.forEach(btn => {
                btn.disabled = false;
            });
            updateDrawButtonState();

            if (selectedShape && selectedShape !== 'marker') {
                drawingTracker.setShapeType(selectedShape);
                drawingTracker.enableDrawing();
            }

            preparePopup.classList.remove('active');
        } catch (e) {
            console.error(e);
            executeBtn.disabled = false;
            prepareBtn.disabled = false;
        }
    });


    // /////////////////////////////////////////////
    //           HEAT SETTINGS
    // /////////////////////////////////////////////

    function updateAverageHeat(heat: number | null | undefined) {
        if (!averageHeatDisplay) return; // safety check
        if (heat === null || heat === undefined || isNaN(heat)) {
            averageHeatDisplay.textContent = 'N/A';
        } else {
            averageHeatDisplay.textContent = `${heat.toFixed(1)}°C`;
        }
    }

    clearMarkersBtn.addEventListener('click', async () => {
        if (drawingTracker) {
            drawingTracker.clearMarkers();
            await drawingTracker.submitHeatMarkers(drawingTracker.getHeatMarkersInVideoSpace());
        }
    });

    clearBtn.addEventListener('click', () => {
        drawingTracker?.clearDrawing();
        drawnShapeType = null;
        updateDrawButtonState();
        if (drawingTracker && selectedShape && selectedShape !== 'marker') {
            drawingTracker.setShapeType(selectedShape);
            drawingTracker.enableDrawing();
        }
    });

    // Fill Accordion Event Listeners
    fillAccordionToggle.addEventListener('click', () => {
        fillSettingsPanel.classList.toggle('open');
        fillAccordionToggle.classList.toggle('active');
        fillEnabled = fillAccordionToggle.classList.contains('active');
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
        await drawingTracker.updateViewSettings(transformedModeSwitch.checked, false);
    });

    window.addEventListener('resize', () => {
        if (drawingTracker) {
            drawingTracker.updateCanvasSize(viewport.offsetWidth, viewport.offsetHeight);
        }
    });
});