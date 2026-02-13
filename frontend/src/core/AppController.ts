//frontend/src/core/AppController.ts

import { CanvasManager } from '../ui/CanvasManager';
import { WebSocketHandler, WebSocketMessage } from '../services/WebSocketHandler';

import { UIRegistry, createUIRegistry } from './UIRegistry';
import { AppState, createAppState } from './AppState';
import { HardwareController } from '../features/hardware/HardwareController';
import { RealTimeDrawing } from '../features/drawing/RealTimeDrawing';
import { ModeManager } from '../features/settings/ModeManager';
import { ToolHandler } from '../features/drawing/ToolHandler';
import { ExecutionManager } from '../features/drawing/ExecutionManager';
import { SettingsManager } from '../features/settings/SettingsManager';
import { PreviewManager } from '../features/drawing/PreviewManager';

// ---------------------------------------------------------------------------
// Global type augmentation – MediaMTXWebRTCReader lives on window at runtime
// ---------------------------------------------------------------------------
declare global {
    interface Window {
        MediaMTXWebRTCReader: any;
    }
}

/**
 * VideoStreamController  (orchestrator)
 *
 * This class is intentionally thin.  Its responsibilities are:
 *   1. Construct every sub-system and wire their dependencies.
 *   2. Run the one-time initialisation sequence.
 *   3. Attach all DOM event listeners (delegating the *handler* logic to
 *      the appropriate sub-system).
 *   4. Own the video → canvas render loop.
 *   5. Receive WebSocket state pushes and fan them out to the UI /
 *      sub-systems that care.
 *
 * Business logic that used to live here has been moved to:
 *   HardwareController   – laser / robot toggles & safety interlocks
 *   RealTimeDrawing      – pointer events & coordinate streaming loop
 *   ModeManager          – Drawing ↔ Thermal ↔ Fixtures switching
 *   ToolHandler          – tool selection, fixture brushes, button state
 *   ExecutionManager     – path execution & shape lifecycle
 *   SettingsManager      – modal, layout persistence, window scaling
 */
class AppController {
    // ---------------------------------------------------------------
    // Shared references  (injected into every sub-system)
    // ---------------------------------------------------------------
    private readonly ui: UIRegistry;
    private readonly state: AppState;

    // ---------------------------------------------------------------
    // Sub-systems
    // ---------------------------------------------------------------
    private readonly wsHandler: WebSocketHandler;
    private readonly hardware: HardwareController;
    private readonly realTime: RealTimeDrawing;
    private readonly modeManager: ModeManager;
    private readonly toolHandler: ToolHandler;
    private readonly executionManager: ExecutionManager;
    private readonly settingsManager: SettingsManager;
    private readonly previewManager: PreviewManager;

    // ---------------------------------------------------------------
    // Late-initialised (created once the video track arrives)
    // ---------------------------------------------------------------
    private canvasManager: CanvasManager | null = null;
    private reader: any = null;   // MediaMTXWebRTCReader instance

    // ---------------------------------------------------------------
    // Constructor
    // ---------------------------------------------------------------
    constructor() {
        // --- shared state ---
        this.ui = createUIRegistry();
        this.state = createAppState();

        // --- WebSocket (no URL yet; connect() is called in init) ---
        this.wsHandler = new WebSocketHandler(null);

        // --- sub-systems ---
        // Many sub-systems need a *live* reference to canvasManager, which is
        // null until the video track arrives.  We hand them a getter so they
        // always see the current value without needing to be re-constructed.
        const getCM = () => this.canvasManager;

        this.hardware = new HardwareController(this.ui, this.state, this.wsHandler);

        this.realTime = new RealTimeDrawing(this.ui, this.state, this.wsHandler);

        this.toolHandler = new ToolHandler(this.ui, this.state, getCM);

        this.modeManager = new ModeManager(
            this.ui,
            this.state,
            getCM,
            () => this.toolHandler.updateDrawButtonState(),
            () => this.toolHandler.updateFixturesButtonState(),
            () => this.toolHandler.updateThermalButtonState(),
        );

        this.previewManager = new PreviewManager(this.ui, this.state, getCM);

        this.executionManager = new ExecutionManager(
            this.ui,
            this.state,
            getCM,
            () => this.toolHandler.updateDrawButtonState(),
            () => this.previewManager,
        );

        this.settingsManager = new SettingsManager(
            this.ui,
            this.state,
            getCM,
            this.hardware,
            () => this.toolHandler.updateDrawButtonState(),
        );

        // TODO: Expose on window for quick console access during development
        // TODO: REMOVE BEFORE SHIPPING
        (window as any).controller = this;

        //Handle resize before initialisation to avoid drawing glitches
        this.settingsManager.handleResize();

        this.init();
    }

    // ===================================================================
    // Initialisation
    // ===================================================================

    private init(): void {
        this.setupVideoCanvas();
        this.setupWebSocket();
        this.bindEvents();
        this.setupInitialState();
    }

    private setupInitialState(): void {
        // Mute is required for autoplay in most browsers
        this.ui.video.muted = true;
        this.ui.video.autoplay = true;

        // Match canvas to container size
        this.ui.canvas.width = this.ui.viewport.offsetWidth;
        this.ui.canvas.height = this.ui.viewport.offsetHeight;

        this.setMessage('Loading stream');

        // Default visual state: Drawing mode active
        document.getElementById('drawingBtn')?.classList.add('active');

        // Restore persisted layout positions
        this.settingsManager.restoreLayoutPositions();

        //Save robot height to localStorage to keep it from shifting on page reload
        const savedHeight = localStorage.getItem('robot_height');
        if (savedHeight) {
            const heightVal = parseInt(savedHeight);
            if (!isNaN(heightVal)) {
                //Update the Slider UI
                this.ui.heightSlider.value = savedHeight;
                //Update the Text Display
                this.ui.heightDisplay.textContent = savedHeight;
                //TODO: May need to send a wshandler.updateState to sync
            }
        }
    }

    private setMessage(str: string): void {
        this.ui.ctx.fillText(str, 10, 50);
    }

    // ===================================================================
    // Network & Video setup
    // ===================================================================

    private setupVideoCanvas(): void {
        this.reader = new window.MediaMTXWebRTCReader({
            url: new URL(`http://${window.location.hostname}:8889/mystream/whep`),
            onError: (err: string) => this.setMessage(err),
            onTrack: (evt: RTCTrackEvent) => {
                if (evt.track.kind === 'video') {
                    this.ui.video.srcObject = evt.streams[0];
                    // Start the per-frame render loop
                    this.ui.video.requestVideoFrameCallback(this.updateCanvasLoop.bind(this));
                    // CanvasManager can now be constructed (video dimensions are known)
                    this.initCanvasManager();
                }
            },
        });
    }

    private setupWebSocket(): void {
        this.wsHandler.onStateUpdate = (newState: WebSocketMessage) => this.syncUiToState(newState);
        this.wsHandler.connect();
    }

    /**
     * (Re-)creates the CanvasManager.  Called once when the first video frame
     * is available.  If a previous instance exists it is disposed first.
     */
    private initCanvasManager(): void {
        if (this.canvasManager) this.canvasManager.dispose();

        this.canvasManager = new CanvasManager(
            this.ui.canvas,
            this.ui.video,
            `http://${window.location.hostname}:443`,
            () => this.executionManager.onShapeComplete(),
            () => this.toolHandler.updateFixturesButtonState(),
            () => this.toolHandler.updateThermalButtonState(),
            () => { this.ui.resetHeatAreaBtn.disabled = false; },
        );

        this.toolHandler.updateDrawButtonState();
        this.toolHandler.updateFixturesButtonState();
        this.toolHandler.updateThermalButtonState();
    }

    // ===================================================================
    // Per-frame render loop
    // ===================================================================

    /**
     * Request the next video frame for the video element behind the canvas and repeat.
     * Fabric handles the rendering of the drawing layer, so related code has been removed from this loop
     */
    private updateCanvasLoop(): void {
        this.ui.video.requestVideoFrameCallback(this.updateCanvasLoop.bind(this));
    }

    /**
     * Central handler for window resize events.
     * Ensures sub-systems update in the correct order to avoid layout thrashing.
     */
    private handleResize(): void {
        // 1. Layout Manager: Calculate new panel positions/sizes
        this.settingsManager.handleResize();

        // 2. Get authoritative viewport dimensions
        const w = this.ui.viewport.offsetWidth;
        const h = this.ui.viewport.offsetHeight;

        // 3. Preview Manager: Resize overlay and re-project path
        this.previewManager.updateOverlaySize();

        // 4. Canvas Manager: Scale fabric canvas and objects
        if (this.canvasManager) {
            // Check against internal canvas dimensions to prevent unnecessary updates
            if (this.ui.canvas.width !== w || this.ui.canvas.height !== h) {
                this.canvasManager.updateCanvasSize(w, h);
            }
        } else {
            // If CM doesn't exist yet, ensure the raw canvas element matches viewport
            this.ui.canvas.width = w;
            this.ui.canvas.height = h;
        }
    }

    // ===================================================================
    // Event binding  (thin wiring – logic lives in sub-systems)
    // ===================================================================

    private bindEvents(): void {
        // --- Popups & modals ---
        this.ui.settingsBtn.addEventListener('click', () => this.settingsManager.openSettings());
        this.ui.settingsCloseBtn.addEventListener('click', () => this.settingsManager.closeSettings());

        this.ui.prepareBtn.addEventListener('click', () => this.ui.preparePopup.classList.add('active'));
        this.ui.prepareCloseBtn.addEventListener('click', () => this.ui.preparePopup.classList.remove('active'));
        this.ui.prepareCancelBtn.addEventListener('click', () => this.ui.preparePopup.classList.remove('active'));

        this.ui.previewToggleOn.addEventListener('click', () => {
            this.previewManager.togglePreview(true);
        });

        this.ui.previewToggleOff.addEventListener('click', () => {
            this.previewManager.togglePreview(false);
        });

        const refreshPreview = () => {
            if (this.ui.previewToggleOn.classList.contains('active')) {
                this.previewManager.refreshPreview();
            }
        };

        this.ui.rasterBtnA.addEventListener('click', refreshPreview);
        this.ui.rasterBtnB.addEventListener('click', refreshPreview);

        let debounceTimer: any;
        this.ui.rasterDensityInput.addEventListener('input', () => {
            clearTimeout(debounceTimer);
            debounceTimer = setTimeout(refreshPreview, 500);
        });


        // Clicking the background overlay closes whichever modal is open
        this.ui.overlay.addEventListener('click', () => {
            if (this.ui.settingsPopup.classList.contains('active')) this.settingsManager.closeSettings();
            if (this.ui.preparePopup.classList.contains('active')) this.ui.preparePopup.classList.remove('active');
        });

        // --- Settings sidebar tab navigation ---
        this.ui.sidebarButtons.forEach(btn => {
            btn.addEventListener('click', () => {
                const targetId = btn.getAttribute('data-target');
                if (!targetId) return;

                this.ui.sidebarButtons.forEach(b => b.classList.remove('active'));
                this.ui.settingsPanels.forEach(p => p.classList.remove('active'));
                btn.classList.add('active');
                document.getElementById(targetId)?.classList.add('active');
            });
        });

        // --- Real-time drawing pointer events ---
        this.ui.realTimePen.addEventListener('click', () => this.toolHandler.handleRealTimeToolSelection(this.ui.realTimePen, 'pen'));
        this.ui.viewport.addEventListener('pointerdown', (e) => this.realTime.handleStart(e));
        this.ui.viewport.addEventListener('pointermove', (e) => this.realTime.handleMove(e));
        this.ui.viewport.addEventListener('pointerup', (e) => this.realTime.handleEnd(e));
        this.ui.viewport.addEventListener('pointercancel', (e) => this.realTime.handleEnd(e));

        // --- Processing mode toggle (real-time ↔ batch) ---
        this.ui.processingModeSwitch.addEventListener('change', () => this.modeManager.toggleMode());

        // --- Top-level mode buttons (Drawing / Thermal / Fixtures) ---
        this.ui.modeButtons.forEach(btn => {
            btn.addEventListener('click', () => {
                this.ui.modeButtons.forEach(b => b.classList.remove('active'));
                btn.classList.add('active');
                this.modeManager.switchMode(btn.id);
            });
        });

        // --- Thermal tools ---
        this.ui.markerBtn.addEventListener('click', () => this.toolHandler.selectThermalTool('marker'));
        this.ui.heatAreaBtn.addEventListener('click', () => {
            if (!this.ui.heatAreaBtn.disabled) this.toolHandler.selectThermalTool('heat');
        });
        this.ui.resetHeatAreaBtn.addEventListener('click', async () => {
            if (!this.canvasManager) return;
            this.ui.resetHeatAreaBtn.disabled = true;
            try { await this.canvasManager.resetHeatArea(); }
            catch (e) { console.error(e); this.ui.resetHeatAreaBtn.disabled = false; }
        });
        this.ui.clearMarkersBtn.addEventListener('click', async () => {
            if (!this.canvasManager) return;
            this.canvasManager.clearMarkers();
            await this.canvasManager.submitHeatMarkers(this.canvasManager.getHeatMarkersInVideoSpace());
        });

        // --- Fixture brushes & actions ---
        this.ui.roundBrushBtn.addEventListener('click', () => this.toolHandler.handleBrushSelection('round'));
        this.ui.squareBrushBtn.addEventListener('click', () => this.toolHandler.handleBrushSelection('square'));
        this.ui.eraserBrushBtn.addEventListener('click', () => this.toolHandler.handleEraserSelection());
        this.ui.brushSizeSlider.addEventListener('input', () => this.toolHandler.handleBrushSizeChange());
        this.ui.clearBoundaryBtn.addEventListener('click', () => this.toolHandler.clearFixtures());
        this.ui.applyFixturesBtn.addEventListener('click', () => this.toolHandler.applyFixtures());

        // --- Hardware toggles (laser / robot) ---
        this.ui.laserBtn.addEventListener('click', () => {
            this.ui.laserBtn.style.pointerEvents = 'none';
            if (this.state.laserConfirmationTimeout) clearTimeout(this.state.laserConfirmationTimeout);
            this.hardware.changeLaserState(!this.ui.laserBtn.classList.contains('active'));
        });
        this.ui.robotBtn.addEventListener('click', () => {
            this.ui.robotBtn.style.pointerEvents = 'none';
            if (this.state.robotConfirmationTimeout) clearTimeout(this.state.robotConfirmationTimeout);
            this.hardware.changeRobotState(!this.ui.robotBtn.classList.contains('active'));
        });

        // Height Slider 
        this.ui.heightSlider.addEventListener('input', () => {
            const heightValue = parseInt(this.ui.heightSlider.value);
            this.ui.heightDisplay.textContent = String(heightValue);
            
            //Save to local storage
            localStorage.setItem('robot_height', String(heightValue));
            
            //Send to backend
            this.wsHandler.updateState({ height: heightValue });
        });

        // --- Shape drawing tools ---
        this.ui.penBtn.addEventListener('click', () => this.toolHandler.handleShapeSelection(this.ui.penBtn, 'freehand'));
        this.ui.squareBtn.addEventListener('click', () => this.toolHandler.handleShapeSelection(this.ui.squareBtn, 'square'));
        this.ui.circleBtn.addEventListener('click', () => this.toolHandler.handleShapeSelection(this.ui.circleBtn, 'circle'));
        this.ui.triangleBtn.addEventListener('click', () => this.toolHandler.handleShapeSelection(this.ui.triangleBtn, 'triangle'));
        this.ui.lineBtn.addEventListener('click', () => this.toolHandler.handleShapeSelection(this.ui.lineBtn, 'line'));

        // Sync button states a tick after mouse/touch interaction on the canvas
        this.ui.canvas.addEventListener('mouseup', () => setTimeout(() => this.toolHandler.updateDrawButtonState(), 50));
        this.ui.canvas.addEventListener('touchend', () => setTimeout(() => this.toolHandler.updateDrawButtonState(), 50));

        // --- Execution actions ---;
        this.ui.executeBtn.addEventListener('click', () => this.executionManager.executePath());
        this.ui.clearBtn.addEventListener('click', () => this.executionManager.clearDrawing());

        // --- Fill / Raster settings ---
        const updateFillState = (isEnabled: boolean) => {
            this.state.fillEnabled = isEnabled;

            if (isEnabled) {
                this.ui.fillOnBtn.classList.add('active');
                this.ui.fillOffBtn.classList.remove('active');
                this.ui.fillSettingsPanel.classList.add('open');
                this.ui.fillSettingsPanel.parentElement?.classList.add('has-open-panel');
            } else {
                this.ui.fillOnBtn.classList.remove('active');
                this.ui.fillOffBtn.classList.add('active');
                this.ui.fillSettingsPanel.classList.remove('open');
                this.ui.fillSettingsPanel.parentElement?.classList.remove('has-open-panel');
            }
            refreshPreview();
        };

        this.ui.fillOnBtn.addEventListener('click', () => updateFillState(true));
        this.ui.fillOffBtn.addEventListener('click', () => updateFillState(false));

        // Raster pattern buttons are mutually exclusive
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

        // --- View-transform  ---
        this.ui.transformedModeSwitch.addEventListener('change', async () => {
            if (this.canvasManager) {
                await this.canvasManager.updateViewSettings(
                    this.ui.transformedModeSwitch.checked,
                    false
                );
            }
        });


        // --- Layout position controls ---
        this.ui.layoutTopBtn.addEventListener('click', () => this.settingsManager.setMenuPosition('top'));
        this.ui.layoutBottomBtn.addEventListener('click', () => this.settingsManager.setMenuPosition('bottom'));
        this.ui.layoutLeftBtn.addEventListener('click', () => this.settingsManager.setSidebarPosition('left'));
        this.ui.layoutRightBtn.addEventListener('click', () => this.settingsManager.setSidebarPosition('right'));

        // --- Window resize ---
        window.addEventListener('resize', () => {
            this.handleResize();
        });
    }

    // ===================================================================
    // WebSocket state → UI sync
    // ===================================================================

    /**
     * Fan-out: applies every field the server might push down to the
     * correct piece of UI or sub-system.
     */
    private syncUiToState(state: Partial<WebSocketMessage>): void {
        // --- Robot-position marker ---
        if (state.laserX !== undefined && state.laserY !== undefined) {
            const cw = this.ui.viewport.offsetWidth;
            const ch = this.ui.viewport.offsetHeight;
            const vw = this.ui.video.videoWidth;
            const vh = this.ui.video.videoHeight;

            if (vw > 0 && vh > 0) {
                this.ui.robotMarker.style.left = `${(state.laserX / vw) * cw}px`;
                this.ui.robotMarker.style.top = `${(state.laserY / vh) * ch}px`;
                this.ui.robotMarker.style.display = 'block';
            }
        }

        if (state.path_preview) {
            this.previewManager.handlePathFromWebSocket(state.path_preview);
        }

        // --- Thermal / heat data ---
        if (state.maxHeat !== undefined) {
            this.updateMaxHeat(state.maxHeat);
        }
        if (state.heat_markers && this.canvasManager) {
            this.canvasManager.updateMarkerTemperatures(state.heat_markers);
        }

        // --- Hardware state (delegated to HardwareController) ---
        if (state.isLaserOn !== undefined) {
            this.hardware.applyServerLaserState(!!state.isLaserOn);
        }
        if (state.isRobotOn !== undefined) {
            this.hardware.applyServerRobotState(!!state.isRobotOn);
        }
    }

    /** Formats the heat value and writes it into the thermal display element. */
    private updateMaxHeat(heat: number | null | undefined): void {
        if (!this.ui.maxHeatDisplay) return;
        this.ui.maxHeatDisplay.textContent =
            (heat === null || heat === undefined || isNaN(heat))
                ? 'N/A'
                : `${heat.toFixed(1)}°C`;
    }
}

// ===========================================================================
// Bootstrap
// ===========================================================================
window.addEventListener('load', () => {
    new AppController();
});