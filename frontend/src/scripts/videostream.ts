import { DrawingTracker } from './drawing/DrawingTracker';
import { ShapeType } from './types';
import { WebSocketHandler, WebSocketMessage } from './api/WebSocketHandler';

// --- Global Type Definitions ---
declare global {
    interface Window {
        // External WebRTC reader library for low-latency streaming
        MediaMTXWebRTCReader: any;
    }
}

/**
 * VideoStreamController
 * * Manages the main application logic, bridging the UI, Video Stream, 
 * Drawing Logic (Canvas), and WebSocket communication with the robot backend.
 */
class VideoStreamController {

    // =========================================
    // SECTION: UI References
    // =========================================
    // Centralized cache of all DOM elements to avoid repeated getElementById calls.
    private ui = {
        // --- Core Viewport ---
        viewport: document.getElementById('viewport') as HTMLElement,
        video: document.getElementById('video') as HTMLVideoElement,
        canvas: document.getElementById('canvas') as HTMLCanvasElement,
        ctx: (document.getElementById('canvas') as HTMLCanvasElement).getContext("2d")!,

        // --- Modals & Overlays ---
        overlay: document.getElementById('overlay') as HTMLElement,
        settingsPopup: document.getElementById('settingsPopup') as HTMLElement,
        preparePopup: document.getElementById('preparePopup') as HTMLElement,


        // --- Action Buttons ---
        settingsBtn: document.getElementById('settingsBtn') as HTMLButtonElement,
        settingsCloseBtn: document.getElementById('settingsCloseBtn') as HTMLButtonElement,
        prepareBtn: document.getElementById('prepareBtn') as HTMLButtonElement,
        prepareCloseBtn: document.getElementById('prepareCloseBtn') as HTMLButtonElement,
        prepareCancelBtn: document.getElementById('prepareCancelBtn') as HTMLButtonElement,
        executeBtn: document.getElementById('executeBtn') as HTMLButtonElement,
        clearBtn: document.getElementById('clearBtn') as HTMLButtonElement,

        // --- Hardware Controls (Robot/Laser) ---
        robotBtn: document.getElementById('robot-toggle-container') as HTMLButtonElement,
        laserBtn: document.getElementById('laser-toggle-container') as HTMLButtonElement,

        // --- View & Mode Toggles ---
        processingModeSwitch: document.getElementById('processing-mode') as HTMLInputElement, // Real-time vs Batch
        transformedModeSwitch: document.getElementById('transformed-view-mode') as HTMLInputElement,
        saveView: document.getElementById('save-view') as HTMLInputElement,
        layoutTopBtn: document.getElementById('layout-top') as HTMLButtonElement,
        layoutBottomBtn: document.getElementById('layout-bottom') as HTMLButtonElement,
        layoutLeftBtn: document.getElementById('layout-left') as HTMLButtonElement,
        layoutRightBtn: document.getElementById('layout-right') as HTMLButtonElement,

        // --- Input Fields ---
        speedInput: document.getElementById('speedInput') as HTMLInputElement,

        // --- Element Groups (NodeLists) ---
        batchUiElements: document.querySelectorAll('.batch-ui'), // Elements hidden during real-time mode
        toggleButtons: document.querySelectorAll('#middle-icon-section .icon-btn') as NodeListOf<HTMLButtonElement>,
        sidebarButtons: document.querySelectorAll('.settings-sidebar .sidebar-btn') as NodeListOf<HTMLButtonElement>,
        settingsPanels: document.querySelectorAll('.settings-main .settings-panel') as NodeListOf<HTMLElement>,

        // --- Visual Markers ---
        robotMarker: document.getElementById('robot-marker') as HTMLElement, // The red dot showing robot pos

        // --- Shape Drawing Tools ---
        penBtn: document.getElementById('penBtn') as HTMLButtonElement,
        squareBtn: document.getElementById('squareBtn') as HTMLButtonElement,
        circleBtn: document.getElementById('circleBtn') as HTMLButtonElement,
        triangleBtn: document.getElementById('triangleBtn') as HTMLButtonElement,
        lineBtn: document.getElementById('lineBtn') as HTMLButtonElement,
        markerBtn: document.getElementById('markerBtn') as HTMLButtonElement,
        clearMarkersBtn: document.getElementById('clearMarkersBtn') as HTMLButtonElement,

        // --- Raster/Fill Settings ---
        fillAccordionToggle: document.getElementById('fillAccordionToggle') as HTMLButtonElement,
        fillSettingsPanel: document.getElementById('fillSettingsPanel') as HTMLElement,
        rasterBtnA: document.getElementById('rasterA') as HTMLButtonElement,
        rasterBtnB: document.getElementById('rasterB') as HTMLButtonElement,
        rasterDensityInput: document.getElementById('densityRaster') as HTMLInputElement,

        // --- Virtual Fixtures (Boundaries) ---
        fixturesTools: document.getElementById('fixtures-tools') as HTMLElement,
        drawingTools: document.getElementById('drawing-tools') as HTMLElement,
        thermalTools: document.getElementById('thermal-tools') as HTMLElement,

        roundBrushBtn: document.getElementById('roundBrushBtn') as HTMLButtonElement,
        squareBrushBtn: document.getElementById('squareBrushBtn') as HTMLButtonElement,
        brushSizeSlider: document.getElementById('brushSizeSlider') as HTMLInputElement,
        clearBoundaryBtn: document.getElementById('clearBoundaryBtn') as HTMLButtonElement,
        applyFixturesBtn: document.getElementById('applyFixturesBtn') as HTMLButtonElement,
        eraserBrushBtn: document.getElementById('eraserBtn') as HTMLButtonElement,

        // Groups for toggling visibility based on mode
        fixturesUiElements: document.querySelectorAll('.fixtures-ui-only'),
        drawingUiElements: document.querySelectorAll('.drawing-ui-only'),
        thermalUiElements: document.querySelectorAll('.thermal-ui'),

        // --- Thermal Data ---
        averageHeatDisplay: document.getElementById('average-heat-display') as HTMLElement,
        heatAreaBtn: document.getElementById('heatAreaBtn') as HTMLButtonElement,
        resetHeatAreaBtn: document.getElementById('resetHeatAreaBtn') as HTMLButtonElement,

        // --- Main Mode Switchers ---
        modeButtons: document.querySelectorAll('.mode-btn')
    };

    // =========================================
    // SECTION: Internal State
    // =========================================
    private state = {
        // Timeouts to prevent rapid toggling of hardware buttons
        laserConfirmationTimeout: null as number | null,
        robotConfirmationTimeout: null as number | null,

        // Drawing State
        selectedShape: null as ShapeType | 'marker' | null, // Tool currently selected
        drawnShapeType: null as ShapeType | null,           // Shape currently on canvas

        // Real-time Mode State
        isRealTimeDrawing: false,
        latestRealTimePos: null as { x: number, y: number } | null,

        // Raster/Fill State
        fillEnabled: false,
        selectedRasterPattern: 'line_raster' as 'line_raster' | 'spiral_raster' | null,

        // Application Mode
        currentMode: 'drawing' as 'drawing' | 'thermal' | 'fixtures',

        // Fixture Brush State
        selectedBrushType: null as 'round' | 'square' | null,
        isEraserActive: false
    };

    // --- Sub-Systems ---
    private wsHandler: WebSocketHandler;        // Handles JSON messaging with server
    private drawingTracker: DrawingTracker | null = null; // Handles Fabric.js canvas logic
    private reader: any = null;                 // Handles WebRTC video stream
    private readonly TARGET_WIDTH = 1700;

    constructor() {
        this.wsHandler = new WebSocketHandler(null);
        (window as any).controller = this;
        this.init();
    }

    // =========================================
    // SECTION: Initialization
    // =========================================
    private init() {
        this.setupVideoCanvas();
        this.setupWebSocket();
        this.bindEvents();
        this.setupInitialState();
        this.handleResize();
    }

    private setupInitialState() {
        // Mute is required for many browsers to allow autoplay
        this.ui.video.muted = true;
        this.ui.video.autoplay = true;

        // Match canvas dimensions to the container
        this.ui.canvas.width = this.ui.viewport.offsetWidth;
        this.ui.canvas.height = this.ui.viewport.offsetHeight;

        this.setMessage("Loading stream");

        // Default to Drawing mode visually
        document.getElementById('drawingBtn')?.classList.add('active');

        const savedMenuPosition = localStorage.getItem('menuPosition') || 'top';
        const savedSidebarPosition = localStorage.getItem('sidebarPosition') || 'left';

        if (savedMenuPosition === 'bottom') {
            document.body.classList.add('menu-bottom');
            this.ui.layoutBottomBtn.classList.add('active');
            this.ui.layoutTopBtn.classList.remove('active');
        }

        if (savedSidebarPosition === 'right') {
            document.body.classList.add('sidebar-right');
            this.ui.layoutRightBtn.classList.add('active');
            this.ui.layoutLeftBtn.classList.remove('active');
        }
    }

    private setMessage(str: string) {
        this.ui.ctx.fillText(str, 10, 50);
    }

    // =========================================
    // SECTION: Network & Video Setup
    // =========================================

    private setupWebSocket() {
        // When server sends state (robot pos, heat, etc.), update the UI
        this.wsHandler.onStateUpdate = (newState: WebSocketMessage) => this.syncUiToState(newState);
        this.wsHandler.connect();
    }

    private setupVideoCanvas() {
        // Initialize external WHEP reader for low-latency video
        this.reader = new window.MediaMTXWebRTCReader({
            url: new URL(`http://${window.location.hostname}:8889/mystream/whep`),
            onError: (err: string) => this.setMessage(err),
            onTrack: (evt: RTCTrackEvent) => {
                if (evt.track.kind === 'video') {
                    // Attach stream to video element
                    this.ui.video.srcObject = evt.streams[0];

                    // Start the render loop synchronized with video frames
                    this.ui.video.requestVideoFrameCallback(this.updateCanvasLoop.bind(this));

                    // Initialize the drawing layer now that video size is known
                    this.initDrawingTracker();
                }
            },
        });
    }

    private initDrawingTracker() {
        if (this.drawingTracker) this.drawingTracker.dispose();

        // DrawingTracker manages the Fabric.js layer on top of the video
        this.drawingTracker = new DrawingTracker(
            this.ui.canvas,
            this.ui.video,
            `http://${window.location.hostname}:443`,
            () => this.onShapeComplete(),      // Callback when a shape is finished
            () => this.updateFixturesButtonState(),
            () => this.updateThermalButtonState(),
            () => { this.ui.resetHeatAreaBtn.disabled = false; }
        );

        this.updateDrawButtonState();
        this.updateFixturesButtonState();
        this.updateThermalButtonState();
    }

    /**
     * Main Render Loop
     * Draws the video frame onto the canvas, then lets DrawingTracker draw overlay shapes.
     * Uses requestVideoFrameCallback for optimal synchronization with video refresh rate.
     */
    private updateCanvasLoop() {
        this.ui.ctx.drawImage(this.ui.video, 0, 0, this.ui.canvas.width, this.ui.canvas.height);

        // Only render Fabric.js overlay if we are NOT in Real-Time mode
        // (In Real-Time mode, the loop handles rendering differently to reduce latency)
        if (this.ui.processingModeSwitch && !this.ui.processingModeSwitch.checked && this.drawingTracker) {
            this.drawingTracker.render();
        }

        this.ui.video.requestVideoFrameCallback(this.updateCanvasLoop.bind(this));
    }

    // =========================================
    // SECTION: Event Handlers
    // =========================================

    private bindEvents() {
        // --- Popups & Modals ---
        this.ui.settingsBtn.addEventListener('click', () => this.openSettings());
        this.ui.settingsCloseBtn.addEventListener('click', () => this.closeSettings());

        // Prepare/Execute Workflow
        this.ui.prepareBtn.addEventListener('click', () => this.ui.preparePopup.classList.add('active'));
        this.ui.prepareCloseBtn.addEventListener('click', () => this.ui.preparePopup.classList.remove('active'));
        this.ui.prepareCancelBtn.addEventListener('click', () => this.ui.preparePopup.classList.remove('active'));

        // Close modals when clicking background overlay
        this.ui.overlay.addEventListener('click', () => {
            if (this.ui.settingsPopup.classList.contains('active')) this.closeSettings();
            if (this.ui.preparePopup.classList.contains('active')) this.ui.preparePopup.classList.remove('active');
        });

        // --- Settings Sidebar Navigation ---
        this.ui.sidebarButtons.forEach(btn => {
            btn.addEventListener('click', () => {
                const targetId = btn.getAttribute('data-target');
                if (!targetId) return;

                // Toggle active class on sidebar buttons and corresponding panels
                this.ui.sidebarButtons.forEach(b => b.classList.remove('active'));
                this.ui.settingsPanels.forEach(p => p.classList.remove('active'));
                btn.classList.add('active');
                document.getElementById(targetId)?.classList.add('active');
            });
        });

        // --- Real-time Drawing Interactions ---
        this.ui.viewport.addEventListener('pointerdown', this.handleRealTimeStart.bind(this));
        this.ui.viewport.addEventListener('pointermove', (e) => {
            if (this.state.isRealTimeDrawing) {
                e.preventDefault();
                // Store raw coordinates; loop will process them
                this.state.latestRealTimePos = this.getCanvasCoordinates(e.clientX, e.clientY);
            }
        });
        this.ui.viewport.addEventListener('pointerup', this.handleRealTimeEnd.bind(this));
        this.ui.viewport.addEventListener('pointercancel', this.handleRealTimeEnd.bind(this));

        // --- Mode & View Controls ---
        this.ui.processingModeSwitch.addEventListener('change', () => this.toggleMode());
        this.ui.modeButtons.forEach(btn => {
            btn.addEventListener('click', () => {
                this.ui.modeButtons.forEach(b => b.classList.remove('active'));
                btn.classList.add('active');
                this.switchMode(btn.id);
            });
        });

        // --- Thermal Tools ---
        this.ui.markerBtn.addEventListener('click', () => this.selectThermalTool('marker'));
        this.ui.heatAreaBtn.addEventListener('click', () => {
            if (!this.ui.heatAreaBtn.disabled) this.selectThermalTool('heat');
        });
        this.ui.resetHeatAreaBtn.addEventListener('click', async () => {
            if (!this.drawingTracker) return;
            this.ui.resetHeatAreaBtn.disabled = true;
            try { await this.drawingTracker.resetHeatArea(); }
            catch (e) { console.error(e); this.ui.resetHeatAreaBtn.disabled = false; }
        });

        // --- Fixture Brushes ---
        this.ui.roundBrushBtn.addEventListener('click', () => this.handleBrushSelection('round'));
        this.ui.squareBrushBtn.addEventListener('click', () => this.handleBrushSelection('square'));
        this.ui.eraserBrushBtn.addEventListener('click', () => this.handleEraserSelection());

        this.ui.brushSizeSlider.addEventListener('input', () => {
            if (this.state.selectedBrushType && this.drawingTracker) {
                this.drawingTracker.setFixturesBrush(
                    this.state.selectedBrushType,
                    parseInt(this.ui.brushSizeSlider.value),
                    this.state.isEraserActive
                );
            }
        });

        // --- Fixture Actions ---
        this.ui.clearBoundaryBtn.addEventListener('click', () => this.clearFixtures());
        this.ui.applyFixturesBtn.addEventListener('click', () => this.applyFixtures());

        // --- Hardware Toggles (Laser/Robot) ---
        this.ui.laserBtn.addEventListener('click', () => {
            // Disable interaction temporarily to await server confirmation
            this.ui.laserBtn.style.pointerEvents = 'none';
            if (this.state.laserConfirmationTimeout) clearTimeout(this.state.laserConfirmationTimeout);
            this.changeLaserState(!this.ui.laserBtn.classList.contains('active'));
        });
        this.ui.robotBtn.addEventListener('click', () => {
            this.ui.robotBtn.style.pointerEvents = 'none';
            if (this.state.robotConfirmationTimeout) clearTimeout(this.state.robotConfirmationTimeout);
            this.changeRobotState(!this.ui.robotBtn.classList.contains('active'));
        });

        // --- Drawing Tools (Shapes) ---
        this.ui.penBtn.addEventListener('click', () => this.handleShapeSelection(this.ui.penBtn, 'freehand'));
        this.ui.squareBtn.addEventListener('click', () => this.handleShapeSelection(this.ui.squareBtn, 'square'));
        this.ui.circleBtn.addEventListener('click', () => this.handleShapeSelection(this.ui.circleBtn, 'circle'));
        this.ui.triangleBtn.addEventListener('click', () => this.handleShapeSelection(this.ui.triangleBtn, 'triangle'));
        this.ui.lineBtn.addEventListener('click', () => this.handleShapeSelection(this.ui.lineBtn, 'line'));

        // Update button states shortly after interaction to ensure sync
        this.ui.canvas.addEventListener('mouseup', () => setTimeout(() => this.updateDrawButtonState(), 50));
        this.ui.canvas.addEventListener('touchend', () => setTimeout(() => this.updateDrawButtonState(), 50));

        // --- Execution Actions ---
        this.ui.executeBtn.addEventListener('click', () => this.executePath());
        this.ui.clearBtn.addEventListener('click', () => this.clearDrawing());
        this.ui.clearMarkersBtn.addEventListener('click', async () => {
            if (this.drawingTracker) {
                this.drawingTracker.clearMarkers();
                await this.drawingTracker.submitHeatMarkers(this.drawingTracker.getHeatMarkersInVideoSpace());
            }
        });

        // --- Fill/Raster Settings ---
        this.ui.fillAccordionToggle.addEventListener('click', () => {
            this.ui.fillSettingsPanel.classList.toggle('open');
            this.ui.fillAccordionToggle.classList.toggle('active');
            this.state.fillEnabled = this.ui.fillAccordionToggle.classList.contains('active');
        });

        // Raster Pattern Selection (Mutual Exclusion)
        this.ui.rasterBtnA.addEventListener('click', () => {
            this.ui.rasterBtnA.classList.add('active');
            this.ui.rasterBtnB.classList.remove('active');
            this.state.selectedRasterPattern = 'line_raster';
        });
        this.ui.rasterBtnB.addEventListener('click', () => {
            this.ui.rasterBtnB.classList.add('active');
            this.ui.rasterBtnA.classList.remove('active');
            this.state.selectedRasterPattern = 'spiral_raster';
        });

        // --- View Transforms ---
        this.ui.saveView.addEventListener('click', async () => {
            if (this.drawingTracker) {
                // Send transform settings to DrawingTracker
                await this.drawingTracker.updateViewSettings(this.ui.transformedModeSwitch.checked, false);
            }
        });

        // --- Menu Position Controls ---
        this.ui.layoutTopBtn.addEventListener('click', () => this.setMenuPosition('top'));
        this.ui.layoutBottomBtn.addEventListener('click', () => this.setMenuPosition('bottom'));

        // --- Sidebar Position Controls ---
        this.ui.layoutLeftBtn.addEventListener('click', () => this.setSidebarPosition('left'));
        this.ui.layoutRightBtn.addEventListener('click', () => this.setSidebarPosition('right'));

        // Handle window resizing
        window.addEventListener('resize', () => {
            this.handleResize();

            if (this.drawingTracker) {
                const newWidth = this.ui.viewport.offsetWidth;
                const newHeight = this.ui.viewport.offsetHeight;

                if (this.ui.canvas.width !== newWidth || this.ui.canvas.height !== newHeight) {
                    this.drawingTracker.updateCanvasSize(newWidth, newHeight);
                }
            }
        });
    }

    // =========================================
    // SECTION: Logic & State Synchronization
    // =========================================

    /**
     * Updates the UI based on incoming WebSocket state from the robot/server.
     */
    private syncUiToState(state: Partial<WebSocketMessage>) {
        //Update Robot Position Marker
        if (state.laserX !== undefined && state.laserY !== undefined) {
            const containerWidth = this.ui.viewport.offsetWidth;
            const containerHeight = this.ui.viewport.offsetHeight;
            const videoWidth = this.ui.video.videoWidth;
            const videoHeight = this.ui.video.videoHeight;

            if (videoWidth > 0 && videoHeight > 0) {
                const cssLeft = (state.laserX / videoWidth) * containerWidth;
                const cssTop = (state.laserY / videoHeight) * containerHeight;
                this.ui.robotMarker.style.left = `${cssLeft}px`;
                this.ui.robotMarker.style.top = `${cssTop}px`;
                this.ui.robotMarker.style.display = 'block';
            }
        }

        //Update Thermal/Heat Data
        if (state.averageHeat !== undefined) this.updateAverageHeat(state.averageHeat);
        if (state.heat_markers && this.drawingTracker) this.drawingTracker.updateMarkerTemperatures(state.heat_markers);

        //Update Hardware Toggles
        if (state.isLaserOn !== undefined) {
            const isOn = !!state.isLaserOn;

            //Always update the visual state to match the server
            this.ui.laserBtn.classList.toggle('active', isOn);

            // If we were waiting for confirmation, clear the lock immediately
            if (this.state.laserConfirmationTimeout) {
                clearTimeout(this.state.laserConfirmationTimeout);
                this.state.laserConfirmationTimeout = null;
                this.ui.laserBtn.style.pointerEvents = 'auto';
            }
        }

        if (state.isRobotOn !== undefined) {
            const isOn = !!state.isRobotOn;

            //Always update the visual state to match the server
            this.ui.robotBtn.classList.toggle('active', isOn);

            //If we were waiting for confirmation, clear the lock immediately
            if (this.state.robotConfirmationTimeout) {
                clearTimeout(this.state.robotConfirmationTimeout);
                this.state.robotConfirmationTimeout = null;
                this.ui.robotBtn.style.pointerEvents = 'auto';
            }
        }
    }

    private updateAverageHeat(heat: number | null | undefined) {
        if (!this.ui.averageHeatDisplay) return;
        this.ui.averageHeatDisplay.textContent = (heat === null || heat === undefined || isNaN(heat)) ? 'N/A' : `${heat.toFixed(1)}°C`;
    }

    /**
     * Converts Mouse/Touch client coordinates to Canvas-relative coordinates.
     */
    private getCanvasCoordinates(clientX: number, clientY: number) {
        const rect = this.ui.canvas.getBoundingClientRect();
        return { x: clientX - rect.left, y: clientY - rect.top };
    }

    // =========================================
    // SECTION: Real-Time Drawing Mode
    // =========================================

    private handleRealTimeStart(e: PointerEvent) {
        // Only active if switch is ON and Pen tool is selected
        if (!this.ui.processingModeSwitch.checked || this.state.selectedShape !== 'freehand') return;

        e.preventDefault();
        this.ui.canvas.setPointerCapture(e.pointerId); // Capture pointer for dragging outside canvas

        this.state.isRealTimeDrawing = true;
        this.state.latestRealTimePos = this.getCanvasCoordinates(e.clientX, e.clientY);

        // Notify server that a path is starting
        this.wsHandler.updateState({ pathEvent: 'start' });
        this.runRealTimeLoop();
    }

    /**
     * High-frequency loop for sending coordinates during real-time drawing.
     * Separated from the video render loop to minimize input latency.
     */
    private runRealTimeLoop() {
        if (!this.state.isRealTimeDrawing) return;

        if (this.state.latestRealTimePos) {
            // Normalize Screen coords -> Video coords for server
            const vidX = (this.state.latestRealTimePos.x / this.ui.canvas.width) * this.ui.video.videoWidth;
            const vidY = (this.state.latestRealTimePos.y / this.ui.canvas.height) * this.ui.video.videoHeight;

            this.wsHandler.updateState({ x: vidX, y: vidY });
        }
        requestAnimationFrame(this.runRealTimeLoop.bind(this));
    }

    private handleRealTimeEnd(e: PointerEvent) {
        if (!this.state.isRealTimeDrawing) return;

        e.preventDefault();
        this.ui.canvas.releasePointerCapture(e.pointerId);

        this.state.isRealTimeDrawing = false;
        this.state.latestRealTimePos = null;
        this.wsHandler.updateState({ pathEvent: 'end' });
    }

    // =========================================
    // SECTION: Mode Switching Logic
    // =========================================

    /**
     * Toggles between Real-Time (streaming coords) and Batch (shape drawing) modes.
     */
    private toggleMode() {
        const isRealTime = this.ui.processingModeSwitch.checked;

        if (isRealTime) {
            this.ui.batchUiElements.forEach(el => el.classList.add('hidden-mode'));
            this.drawingTracker?.disableDrawing();
        } else {
            this.ui.batchUiElements.forEach(el => el.classList.remove('hidden-mode'));
            this.drawingTracker?.disableDrawing();
        }

        // Reset tool selections
        this.ui.toggleButtons.forEach(btn => {
            btn.classList.remove('selected');
            btn.disabled = false;
        });
        this.state.selectedShape = null;
        this.state.drawnShapeType = null;
        this.updateDrawButtonState();
    }

    /**
     * Handles switching between main application tabs: Drawing, Thermal, Fixtures.
     * Manages teardown of the old mode and setup of the new mode.
     */
    private switchMode(modeId: string) {
        console.log("Switching to mode:", modeId);

        // --- TEARDOWN: Cleanup Previous Mode ---
        if (this.state.currentMode === 'fixtures' && modeId !== 'fixturesBtn') {
            // If leaving Fixtures mode, clear temporary brushes
            if (this.drawingTracker && this.drawingTracker.canApplyFixtures()) {
                this.drawingTracker.clearFixtures();
                this.state.selectedBrushType = null;
                this.state.isEraserActive = false;
                this.ui.roundBrushBtn.classList.remove('selected');
                this.ui.squareBrushBtn.classList.remove('selected');
                this.ui.eraserBrushBtn.classList.remove('selected');
            }
        }
        if (this.state.currentMode === 'thermal' && modeId !== 'thermalBtn') {
            this.drawingTracker?.disableMarkerMode();
            this.ui.markerBtn.classList.remove('selected');
        }

        // --- SETUP: Activate New Mode ---

        // CASE 1: Fixtures Mode
        if (modeId === 'fixturesBtn') {
            this.state.currentMode = 'fixtures';

            // Toggle UI Visibility
            this.ui.drawingTools.classList.add('hidden');
            this.ui.thermalTools.classList.add('hidden');
            this.ui.fixturesTools.classList.remove('hidden');

            this.ui.thermalUiElements.forEach(el => el.classList.add('hidden'));
            this.ui.drawingUiElements.forEach(el => el.classList.add('hidden'));
            this.ui.fixturesUiElements.forEach(el => el.classList.remove('hidden'));

            // Toggle Tracker Logic
            this.drawingTracker?.disableDrawing();
            this.drawingTracker?.enableFixturesMode();
            this.updateFixturesButtonState();

            // CASE 2: Thermal Mode
        } else if (modeId === 'thermalBtn') {
            this.state.currentMode = 'thermal';

            this.drawingTracker?.showMarkers();
            this.updateThermalButtonState();

            // Toggle UI Visibility
            this.ui.drawingTools.classList.add('hidden');
            this.ui.thermalTools.classList.remove('hidden');
            this.ui.fixturesTools.classList.add('hidden');

            this.ui.drawingUiElements.forEach(el => el.classList.add('hidden'));
            this.ui.fixturesUiElements.forEach(el => el.classList.add('hidden'));
            this.drawingTracker?.disableFixturesMode();
            this.drawingTracker?.disableDrawing();

            // Keep fixtures visible but not editable
            if (this.drawingTracker?.hasFixtures()) this.drawingTracker?.showFixtures();
            this.ui.thermalUiElements.forEach(el => el.classList.remove('hidden'));

            // Restore marker tool if it was selected
            if (this.state.selectedShape === 'marker') {
                this.ui.markerBtn.classList.add('selected');
                this.drawingTracker?.enableMarkerMode();
            }

            // CASE 3: Drawing Mode (Default)
        } else {
            this.state.currentMode = 'drawing';

            // Toggle UI Visibility
            this.ui.drawingTools.classList.remove('hidden');
            this.ui.thermalTools.classList.add('hidden');
            this.ui.fixturesTools.classList.add('hidden');
            this.ui.heatAreaBtn.classList.remove('selected');
            this.ui.markerBtn.classList.remove('selected');

            // Toggle Tracker Logic
            this.drawingTracker?.disableMarkerMode();
            this.drawingTracker?.disableHeatAreaMode();

            this.ui.drawingUiElements.forEach(el => el.classList.remove('hidden'));
            this.ui.fixturesUiElements.forEach(el => el.classList.add('hidden'));
            this.ui.thermalUiElements.forEach(el => el.classList.add('hidden'));

            this.drawingTracker?.disableFixturesMode();
            if (this.drawingTracker?.hasFixtures()) this.drawingTracker?.showFixtures();

            // Restore drawing state
            if (this.state.drawnShapeType) {
                // If a shape exists, we can't draw a new one yet
                this.drawingTracker?.disableDrawing();
                this.ui.toggleButtons.forEach(btn => btn.classList.remove('selected'));
                this.highlightShapeBtn(this.state.drawnShapeType);
            } else if (this.state.selectedShape && this.state.selectedShape !== 'marker') {
                // If a tool is selected but no shape drawn, enable drawing
                this.drawingTracker?.setShapeType(this.state.selectedShape);
                this.drawingTracker?.enableDrawing();
                this.ui.toggleButtons.forEach(btn => btn.classList.remove('selected'));
                this.highlightShapeBtn(this.state.selectedShape);
            } else {
                this.drawingTracker?.disableDrawing();
                this.ui.toggleButtons.forEach(btn => btn.classList.remove('selected'));
            }
            this.updateDrawButtonState();
        }
    }

    // Helper to visually select the active shape button
    private highlightShapeBtn(shape: ShapeType) {
        if (shape === 'freehand') this.ui.penBtn.classList.add('selected');
        else if (shape === 'square') this.ui.squareBtn.classList.add('selected');
        else if (shape === 'circle') this.ui.circleBtn.classList.add('selected');
        else if (shape === 'triangle') this.ui.triangleBtn.classList.add('selected');
        else if (shape === 'line') this.ui.lineBtn.classList.add('selected');
    }

    // =========================================
    // SECTION: Tool Handlers (Drawing & Fixtures)
    // =========================================

    private selectThermalTool(tool: 'marker' | 'heat') {
        this.ui.markerBtn.classList.remove('selected');
        this.ui.heatAreaBtn.classList.remove('selected');
        this.drawingTracker?.disableDrawing();
        this.drawingTracker?.disableMarkerMode();
        this.drawingTracker?.disableHeatAreaMode();

        if (tool === 'marker') {
            this.ui.markerBtn.classList.add('selected');
            this.drawingTracker?.enableMarkerMode();
            this.state.selectedShape = 'marker';
        } else {
            this.ui.heatAreaBtn.classList.add('selected');
            this.drawingTracker?.enableHeatAreaMode();
            this.state.selectedShape = null;
        }
    }

    private handleBrushSelection(brushType: 'round' | 'square') {
        if (this.ui.roundBrushBtn.disabled && brushType === 'round') return;
        if (this.ui.squareBrushBtn.disabled && brushType === 'square') return;

        // Toggle off if already selected
        if (this.state.selectedBrushType === brushType) {
            this.ui.roundBrushBtn.classList.remove('selected');
            this.ui.squareBrushBtn.classList.remove('selected');
            this.state.selectedBrushType = null;
            this.state.isEraserActive = false;
            this.drawingTracker?.disableFixturesBrush();
        } else {
            // Activate new brush
            this.ui.roundBrushBtn.classList.remove('selected');
            this.ui.squareBrushBtn.classList.remove('selected');
            this.ui.eraserBrushBtn.classList.remove('selected');

            if (brushType === 'round') this.ui.roundBrushBtn.classList.add('selected');
            else this.ui.squareBrushBtn.classList.add('selected');

            this.state.selectedBrushType = brushType;
            this.state.isEraserActive = false;
            this.drawingTracker?.setFixturesBrush(brushType, parseInt(this.ui.brushSizeSlider.value), false);
        }
        this.updateFixturesButtonState();
    }

    private handleEraserSelection() {
        if (this.ui.eraserBrushBtn.disabled) return;

        if (this.state.isEraserActive) {
            // Toggle off
            this.ui.eraserBrushBtn.classList.remove('selected');
            this.state.selectedBrushType = null;
            this.state.isEraserActive = false;
            this.drawingTracker?.disableFixturesBrush();
        } else {
            // Activate Eraser
            this.ui.roundBrushBtn.classList.remove('selected');
            this.ui.squareBrushBtn.classList.remove('selected');
            this.ui.eraserBrushBtn.classList.add('selected');

            this.state.selectedBrushType = 'round'; // Eraser is effectively a round brush
            this.state.isEraserActive = true;
            this.drawingTracker?.setFixturesBrush('round', parseInt(this.ui.brushSizeSlider.value), true);
        }
        this.updateFixturesButtonState();
    }

    private async clearFixtures() {
        if (!this.drawingTracker) return;
        this.ui.clearBoundaryBtn.disabled = true;

        try {
            this.drawingTracker.clearFixtures();
            await this.drawingTracker.clearFixturesOnServer();
        } catch (e) { console.error(e); }

        // Reset brush state after clearing
        this.drawingTracker.disableFixturesBrush();
        this.state.selectedBrushType = null;
        this.state.isEraserActive = false;

        this.ui.roundBrushBtn.classList.remove('selected');
        this.ui.squareBrushBtn.classList.remove('selected');
        this.ui.eraserBrushBtn.classList.remove('selected');
        this.updateFixturesButtonState();
    }

    private async applyFixtures() {
        if (!this.drawingTracker) return;
        this.ui.applyFixturesBtn.disabled = true;
        this.ui.clearBoundaryBtn.disabled = true;

        try {
            // Serialize fixtures and send to backend
            await this.drawingTracker.executeFixtures();

            // Turn off editing brushes
            this.drawingTracker.disableFixturesBrush();
            this.state.selectedBrushType = null;
            this.state.isEraserActive = false;

            this.ui.roundBrushBtn.classList.remove('selected');
            this.ui.squareBrushBtn.classList.remove('selected');
            this.ui.eraserBrushBtn.classList.remove('selected');
            this.updateFixturesButtonState();
        } catch (e) {
            console.error(e);
            this.ui.applyFixturesBtn.disabled = false;
            this.ui.clearBoundaryBtn.disabled = false;
        }
    }

    // =========================================
    // SECTION: Hardware Control (Laser/Robot)
    // =========================================

    private changeLaserState(newState: boolean) {
        this.ui.laserBtn.style.pointerEvents = 'none'; // Lock UI

        const updates: any = { isLaserOn: newState };

        // Safety: If turning OFF laser while Robot is ON, turn Robot OFF too
        if (newState === false && this.ui.robotBtn.classList.contains('active')) {
            updates.isRobotOn = false;
            this.ui.robotBtn.style.pointerEvents = 'none';
            if (this.state.robotConfirmationTimeout) clearTimeout(this.state.robotConfirmationTimeout);
            this.state.robotConfirmationTimeout = setTimeout(() => { this.ui.robotBtn.style.pointerEvents = 'auto'; }, 2000);
        }

        this.wsHandler.updateState(updates);

        // Set safeguard timeout in case server doesn't respond quickly
        if (this.state.laserConfirmationTimeout) clearTimeout(this.state.laserConfirmationTimeout);
        this.state.laserConfirmationTimeout = setTimeout(() => { this.ui.laserBtn.style.pointerEvents = 'auto'; }, 2000);
    }

    private changeRobotState(newState: boolean) {
        this.ui.robotBtn.style.pointerEvents = 'none'; // Lock UI

        const updates: any = { isRobotOn: newState };

        // Safety: If turning OFF robot while Laser is ON, turn Laser OFF too
        if (newState === false && this.ui.laserBtn.classList.contains('active')) {
            updates.isLaserOn = false;
            this.ui.laserBtn.style.pointerEvents = 'none';
            if (this.state.laserConfirmationTimeout) clearTimeout(this.state.laserConfirmationTimeout);
            this.state.laserConfirmationTimeout = setTimeout(() => { this.ui.laserBtn.style.pointerEvents = 'auto'; }, 2000);
        }

        this.wsHandler.updateState(updates);

        // Set safeguard timeout
        if (this.state.robotConfirmationTimeout) clearTimeout(this.state.robotConfirmationTimeout);
        this.state.robotConfirmationTimeout = setTimeout(() => { this.ui.robotBtn.style.pointerEvents = 'auto'; }, 2000);
    }

    // =========================================
    // SECTION: Shape Drawing & Execution
    // =========================================

    private handleShapeSelection(button: HTMLButtonElement, shape: ShapeType | 'marker') {
        // Enforce mode constraints (Real-time only allows freehand)
        if (this.ui.processingModeSwitch.checked && shape !== 'freehand') return;
        if (button.disabled) return;

        if (button.classList.contains('selected')) {
            // Deselect logic
            button.classList.remove('selected');
            this.state.selectedShape = null;
            this.drawingTracker?.disableDrawing();
            this.drawingTracker?.disableMarkerMode();
        } else {
            // Select logic
            this.ui.toggleButtons.forEach(btn => btn.classList.remove('selected'));
            button.classList.add('selected');
            this.state.selectedShape = shape;

            if (this.ui.processingModeSwitch.checked) {
                // Real-time mode: clear old drawings immediately
                this.drawingTracker?.clearDrawing();
                this.state.drawnShapeType = null;
            } else {
                // Batch mode
                if (this.drawingTracker) {
                    if (shape === 'marker') this.drawingTracker.enableMarkerMode();
                    else {
                        this.drawingTracker.setShapeType(shape);
                        this.drawingTracker.enableDrawing();
                    }
                }
            }
        }
        this.updateDrawButtonState();
    }

    private async executePath() {
        if (!this.drawingTracker) return;

        const speed = parseFloat(this.ui.speedInput.value);
        let density = 0;
        if (this.state.fillEnabled) density = parseFloat(this.ui.rasterDensityInput.value);

        if (isNaN(speed) || speed <= 0) { alert("Invalid speed"); return; }

        this.ui.executeBtn.disabled = true;
        this.ui.prepareBtn.disabled = true;

        try {
            // Send path data to backend
            await this.drawingTracker.executePath(speed, String(this.state.selectedRasterPattern), density, this.state.fillEnabled);

            // Clean up UI after successful execution
            this.drawingTracker.clearDrawing();
            this.state.drawnShapeType = null;
            this.ui.toggleButtons.forEach(btn => btn.disabled = false);
            this.updateDrawButtonState();

            // Re-enable drawing tool if one was selected
            if (this.state.selectedShape && this.state.selectedShape !== 'marker') {
                this.drawingTracker.setShapeType(this.state.selectedShape);
                this.drawingTracker.enableDrawing();
            }
            this.ui.preparePopup.classList.remove('active');
        } catch (e) {
            console.error(e);
            this.ui.executeBtn.disabled = false;
            this.ui.prepareBtn.disabled = false;
        }
    }

    private clearDrawing() {
        this.drawingTracker?.clearDrawing();
        this.state.drawnShapeType = null;
        this.updateDrawButtonState();

        // Reset state to allow drawing again immediately
        if (this.drawingTracker && this.state.selectedShape && this.state.selectedShape !== 'marker') {
            this.drawingTracker.setShapeType(this.state.selectedShape);
            this.drawingTracker.enableDrawing();
        }
    }

    private onShapeComplete() {
        // Called by DrawingTracker when user releases mouse after drawing a shape
        if (this.state.selectedShape && this.state.selectedShape !== 'marker') {
            this.state.drawnShapeType = this.state.selectedShape;
            this.updateDrawButtonState();
        }
    }

    // =========================================
    // SECTION: Settings Modal
    // =========================================

    private openSettings() {
        // If a shape exists, clear it before opening settings to avoid state conflicts
        if (this.drawingTracker && this.drawingTracker.hasShape()) {
            this.drawingTracker.clearDrawing();
            this.state.selectedShape = null;
            this.state.drawnShapeType = null;
            this.ui.toggleButtons.forEach(btn => btn.classList.remove('selected'));
            this.updateDrawButtonState();
        }

        // Disable controls while settings are open
        [this.ui.clearBtn, this.ui.prepareBtn, this.ui.robotBtn, this.ui.laserBtn].forEach(btn => btn.disabled = true);

        // Safety: ensure hardware is off
        this.changeLaserState(false);
        this.changeRobotState(false);

        this.ui.settingsPopup.classList.add('active');
        this.ui.overlay.classList.add('active');
    }

    private closeSettings() {
        this.ui.settingsPopup.classList.remove('active');
        this.ui.overlay.classList.remove('active');
        [this.ui.robotBtn, this.ui.laserBtn].forEach(btn => btn.disabled = false);
    }

    private setMenuPosition(position: 'top' | 'bottom') {
        if (position === 'bottom') {
            document.body.classList.add('menu-bottom');
            this.ui.layoutBottomBtn.classList.add('active');
            this.ui.layoutTopBtn.classList.remove('active');
            localStorage.setItem('menuPosition', 'bottom');
        } else {
            document.body.classList.remove('menu-bottom');
            this.ui.layoutTopBtn.classList.add('active');
            this.ui.layoutBottomBtn.classList.remove('active');
            localStorage.setItem('menuPosition', 'top');
        }
    }

    private setSidebarPosition(position: 'left' | 'right') {
        if (position === 'right') {
            document.body.classList.add('sidebar-right');
            this.ui.layoutRightBtn.classList.add('active');
            this.ui.layoutLeftBtn.classList.remove('active');
            localStorage.setItem('sidebarPosition', 'right');
        } else {
            document.body.classList.remove('sidebar-right');
            this.ui.layoutLeftBtn.classList.add('active');
            this.ui.layoutRightBtn.classList.remove('active');
            localStorage.setItem('sidebarPosition', 'left');
        }
    }

    // =========================================
    // SECTION: Button State Management
    // =========================================

    private updateDrawButtonState() {
        const hasShape = this.state.drawnShapeType !== null;

        // Enable action buttons only if a shape is drawn
        this.ui.clearBtn.disabled = !hasShape;
        this.ui.prepareBtn.disabled = !hasShape;
        this.ui.executeBtn.disabled = !hasShape;

        if (hasShape) {
            // Lock other tools if a shape is already present (One shape limit)
            this.ui.penBtn.disabled = (this.state.drawnShapeType !== 'freehand');
            this.ui.squareBtn.disabled = (this.state.drawnShapeType !== 'square');
            this.ui.circleBtn.disabled = (this.state.drawnShapeType !== 'circle');
            this.ui.triangleBtn.disabled = (this.state.drawnShapeType !== 'triangle');
            this.ui.lineBtn.disabled = (this.state.drawnShapeType !== 'line');
            this.ui.markerBtn.disabled = true;
        } else {
            // Unlock all tools
            this.ui.toggleButtons.forEach(btn => btn.disabled = false);
        }
    }

    private updateFixturesButtonState() {
        const hasFixtures = this.drawingTracker?.hasFixtures() ?? false;
        const canApply = this.drawingTracker?.canApplyFixtures() ?? false;

        this.ui.clearBoundaryBtn.disabled = !hasFixtures;
        this.ui.applyFixturesBtn.disabled = !canApply;

        this.ui.roundBrushBtn.disabled = false;
        this.ui.squareBrushBtn.disabled = false;
        this.ui.eraserBrushBtn.disabled = !hasFixtures;
    }

    private updateThermalButtonState() {
        if (!this.drawingTracker) return;
        this.ui.clearMarkersBtn.disabled = !this.drawingTracker.hasMarkers();
    }


    // =========================================
    // SECTION: Window Management
    // =========================================

    private handleResize() {
        const scaler = document.getElementById('app-scaler');
        if (!scaler) return;

        const windowWidth = window.innerWidth;
        const windowHeight = window.innerHeight;

        // If the screen is smaller than your target, scale down
        if (windowWidth < this.TARGET_WIDTH) {
            const scale = windowWidth / this.TARGET_WIDTH;

            // 1. Lock the container width to your desired layout width
            scaler.style.width = `${this.TARGET_WIDTH}px`;

            // 2. Adjust height so it still fills the screen perfectly when scaled
            // (Window Height / Scale) gives us the "virtual" height needed
            scaler.style.height = `${windowHeight / scale}px`;

            // 3. Apply the shrinkage
            scaler.style.transform = `scale(${scale})`;
        } else {
            // Reset to native size if screen is large enough
            scaler.style.width = '100%';
            scaler.style.height = '100%';
            scaler.style.transform = 'none';
        }
    }
}

// Bootstrap
window.addEventListener('load', () => {
    new VideoStreamController();
});