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
import { ShapeType } from '../ui/types';
import { ToastManager } from '../ui/ToastManager';

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

    private isHardwareHeightSynced: boolean = false;
    private isChangingHeight: boolean = false;
    private lastViewportWidth: number = 0;
    private lastViewportHeight: number = 0;

    private zoomLevel: number = 1;
    private panX: number = 0;
    private panY: number = 0;
    private readonly MIN_ZOOM = 1;
    private readonly MAX_ZOOM = 4;

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
        //Add function to clean the state when connection starts
        this.wsHandler.onOpen = () => {
            console.log("WS Connected: Initiating Clean Slate Protocol...");
            this.resetToDefaults();
        };
        this.wsHandler.connect();
    }

    /**
     * CLEAN SLATE PROTOCOL
     * Resets the application to a safe, known state upon connection/refresh.
     */
    private async resetToDefaults(): Promise<void> {
        // --- Turn off hardware ---
        this.hardware.applyServerLaserState(false);
        this.hardware.applyServerRobotState(false);

        // --- Tool reset ---
        document.getElementById('drawingBtn')?.click();
        this.ui.toggleButtons.forEach(btn => btn.classList.remove('selected'));
        this.state.selectedShape = null;
        this.state.drawnShapeType = null;
        this.toolHandler.updateDrawButtonState();

        // --- Transformed view on by default ---
        this.ui.transformedModeSwitch.checked = true;
        this.ui.transformedModeSwitch.dispatchEvent(new Event('change'));
        if (this.canvasManager) {
            await this.canvasManager.updateViewSettings(true, false);
        }

        // --- Height Sync ---
        this.isHardwareHeightSynced = false;
        this.ui.heightTrigger.disabled = true;
        this.ui.heightDisplay.textContent = "...";

        // --- Speed default 10 mm/s ---
        this.ui.speedSlider.value = "10";
        this.ui.speedDisplay.textContent = "10";

        this.wsHandler.updateState({ speed: 10, isLaserOn: false, isRobotOn: false, request_sync: true });

        // --- Reset all masks and the server memory ---
        console.log("Resetting server environment...");
        try {
            await fetch(`http://${window.location.hostname}:443/api/system_reset`, {
                method: 'POST'
            });

            //Wait for CanvasManager to be ready before trying to upload masks!
            const uploadBlankMasks = async () => {
                if (this.canvasManager) {
                    await this.canvasManager.clearFixturesOnServer();
                    await this.canvasManager.resetHeatArea();
                    await this.canvasManager.clearPathAndRasterOnServer();

                    //Clear the local canvas visually once we know it exists
                    this.canvasManager.clearFixtures();
                    this.canvasManager.clearDrawing();
                } else {
                    //Check again in 250ms if the video stream hasn't loaded yet
                    setTimeout(uploadBlankMasks, 250);
                }
            };

            uploadBlankMasks();

        } catch (e) {
            console.warn("Could not reach endpoints to reset masks:", e);
        }
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
            () => { this.ui.resetHeatAreaBtn.disabled = false; this.ui.heatLegend.classList.remove('hidden'); },
        );

        //Force transformed view on init
        this.canvasManager.updateViewSettings(this.ui.transformedModeSwitch.checked, false);

        this.canvasManager.onShapeModified = () => {
            if (this.ui.previewToggleOn.classList.contains('active')) {
                // The preview window is currently open, so silently refresh the data!
                this.previewManager.refreshPreview();
            } else {
                // The preview window is closed. Invalidate old data to wait for new request
                this.executionManager.onShapeComplete();
            }
        };

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
        //Layout Manager: Calculate new panel positions/sizes
        this.settingsManager.handleResize();

        //Get authoritative viewport dimensions
        const w = this.ui.viewport.offsetWidth;
        const h = this.ui.viewport.offsetHeight;

        if (w === 0 || h === 0) return;
        if (w === this.lastViewportWidth && h === this.lastViewportHeight) return;

        this.lastViewportWidth = w;
        this.lastViewportHeight = h;

        //Preview Manager: Resize overlay and re-project path
        this.previewManager.updateOverlaySize();

        //Canvas Manager: Scale fabric canvas and objects
        if (this.canvasManager) {
            //Check against internal canvas dimensions to prevent unnecessary updates
            if (this.ui.canvas.width !== w || this.ui.canvas.height !== h) {
                this.canvasManager.updateCanvasSize(w, h);
            }
        } else {
            //If CM doesn't exist yet, ensure the raw canvas element matches viewport
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
        this.ui.prepareCancelBtn.addEventListener('click', () => {
            this.ui.preparePopup.classList.remove('active');
            this.previewManager.togglePreview(false);
            ToastManager.clearAll();
            this.previewManager.resetPreviewState();
        });

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

        // --- Processing mode toggle ---
        this.ui.processingModeSwitch.addEventListener('change', () => this.modeManager.toggleMode());

        // --- Top-level mode buttons (Drawing / Thermal / Fixtures) ---
        this.ui.modeButtons.forEach(btn => {
            btn.addEventListener('click', () => {
                //If clicking the tab they are already on, do nothing
                if (btn.classList.contains('active')) return;

                //If we are in real time mode and we switch tabs, turn off robot/laser
                if (this.ui.processingModeSwitch.checked) {
                    console.warn("Safety Interlock: In-app tab switched. Disabling hardware.");

                    this.hardware.changeLaserState(false);
                    this.hardware.changeRobotState(false);

                    //Stop the drawing loop if they were mid-stroke while clicking the tab
                    if (this.state.isRealTimeDrawing) {
                        const mockEvent = new PointerEvent('pointerup');
                        this.realTime.handleEnd(mockEvent);
                    }
                }

                // Proceed with normal tab switching
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
            try {
                await this.canvasManager.resetHeatArea();
                this.ui.resetHeatAreaBtn.disabled = true;
                this.ui.heatLegend.classList.add('hidden')

            }
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

        this.ui.heightSlider.addEventListener('mousedown', () => this.isChangingHeight = true);
        this.ui.heightSlider.addEventListener('touchstart', () => this.isChangingHeight = true, { passive: true });

        this.ui.heightSlider.addEventListener('mouseup', () => this.isChangingHeight = false);
        this.ui.heightSlider.addEventListener('touchend', () => this.isChangingHeight = false);

        const togglePopout = (trigger: HTMLElement, flyout: HTMLElement, otherFlyouts: HTMLElement[]) => {
            const isActive = flyout.classList.contains('active');

            // Close any other open flyouts
            otherFlyouts.forEach(f => {
                f.classList.remove('active');
                const t = document.getElementById(f.id.replace('Flyout', 'Trigger'));
                if (t) t.classList.remove('active');
            });

            if (isActive) {
                flyout.classList.remove('active');
                trigger.classList.remove('active');
            } else {
                flyout.classList.add('active');
                trigger.classList.add('active');
            }
        };

        this.ui.speedTrigger.addEventListener('click', (e) => {
            e.stopPropagation(); // Prevent document click from firing
            togglePopout(this.ui.speedTrigger, this.ui.speedFlyout, [this.ui.heightFlyout]);
        });

        this.ui.heightTrigger.addEventListener('click', (e) => {
            e.stopPropagation();
            togglePopout(this.ui.heightTrigger, this.ui.heightFlyout, [this.ui.speedFlyout]);
        });

        // Click outside to close
        document.addEventListener('click', (e) => {
            const target = e.target as HTMLElement;
            if (!target.closest('.popout-control')) {
                this.ui.speedFlyout.classList.remove('active');
                this.ui.speedTrigger.classList.remove('active');
                this.ui.heightFlyout.classList.remove('active');
                this.ui.heightTrigger.classList.remove('active');
            }
        });

        // Speed Slider event (Update text and send to backend)
        this.ui.speedSlider.addEventListener('input', () => {
            this.ui.speedDisplay.textContent = this.ui.speedSlider.value;
        });
        this.ui.speedSlider.addEventListener('change', () => {
            const val = this.ui.speedSlider.value;
            console.log(`Sending final speed command to robot: ${val}`);
            this.wsHandler.updateState({ speed: parseInt(val) });
        });

        // Height Slider 
        this.ui.heightSlider.addEventListener('input', () => {
            // Only update the text on the screen
            this.ui.heightDisplay.textContent = this.ui.heightSlider.value;
        });
        this.ui.heightSlider.addEventListener('change', () => {
            if (!this.isHardwareHeightSynced) return;
            const heightValue = this.ui.heightSlider.value;
            const heightCm = parseFloat(heightValue) * 100;
            const displayVal = heightCm.toFixed(1);
            console.log(`Sending final height command to robot: ${displayVal}`);

            // Send exactly one command to the backend
            this.wsHandler.updateState({ height: parseInt(heightValue) });
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
        this.ui.executeBtn.addEventListener('click', () => {
            // Take a snapshot of the geometry before executing
            if (this.canvasManager && this.state.drawnShapeType) {
                this.canvasManager.backupDrawing();
                this.state.hasBackupShape = true;
                this.state.backupShapeType = this.state.drawnShapeType;
            }
            this.executionManager.executePath();
            // Update button states so the UI knows a backup exists now
            this.toolHandler.updateDrawButtonState();
        });
        this.ui.clearBtn.addEventListener('click', () => this.executionManager.clearDrawing());

        this.ui.restorePathBtn.addEventListener('click', () => {
            if (this.canvasManager && this.state.hasBackupShape) {
                // Restore the state tracker
                this.state.drawnShapeType = this.state.backupShapeType;

                // Ask CanvasManager to rebuild the geometry
                this.canvasManager.restoreDrawing();

                // Lock the tool buttons and disable the restore button
                this.toolHandler.updateDrawButtonState();

                // Highlight the tool button that matches the restored shape
                this.modeManager.highlightShapeBtn(this.state.drawnShapeType as ShapeType);
            }
        });

        this.ui.speedInput.addEventListener('input', () => {
            const val = parseFloat(this.ui.speedInput.value);
            const isValid = !isNaN(val) && val >= 1 && val <= 30;

            if (!isValid) {
                //Highlight the input field red
                this.ui.speedInput.classList.add('input-error');

                //Hard-lock the execute button
                this.ui.executeBtn.disabled = true;
                this.ui.executeBtn.style.pointerEvents = 'none';
                this.ui.executeBtn.style.opacity = '0.3';

                //Lock the preview toggle
                this.ui.previewToggleOn.style.pointerEvents = 'none';
                this.ui.previewToggleOn.style.opacity = '0.3';

                //If preview is currently calculating/open, show an error
                if (this.ui.previewToggleOn.classList.contains('active')) {
                    this.ui.previewDuration.textContent = 'Invalid Speed';
                }

                //Stop any pending auto-refreshes
                clearTimeout(debounceTimer);
            } else {
                //Remove the red highlight
                this.ui.speedInput.classList.remove('input-error');

                //Unlock the preview toggle
                this.ui.previewToggleOn.style.pointerEvents = 'auto';
                this.ui.previewToggleOn.style.opacity = '1';

                //Changing speed changes the kinematics, so the old preview is now void.
                if (this.ui.previewToggleOn.classList.contains('active')) {
                    //Auto-refresh the live preview after they stop typing
                    this.ui.previewDuration.textContent = 'Computing...';
                    this.ui.executeBtn.disabled = true;
                    this.ui.executeBtn.style.pointerEvents = 'none';
                    this.ui.executeBtn.style.opacity = '0.3';

                    clearTimeout(debounceTimer);
                    debounceTimer = setTimeout(refreshPreview, 500);
                } else {
                    //If preview is off, silently void any hidden preview state
                    this.previewManager.resetPreviewState();
                }
            }
        });

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

        //TODO: Remove entirely, unless we want to add more raster patterns in the future.
        /*
        this.ui.rasterBtnA.addEventListener('click', refreshPreview);
        this.ui.rasterBtnB.addEventListener('click', refreshPreview);

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
        */

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

        //DISABLE DEFAULT BROWSER ZOOM
        document.addEventListener('wheel', (e: WheelEvent) => {
            if (e.ctrlKey || e.metaKey) {
                e.preventDefault();
            }
        }, { passive: false });

        document.addEventListener('keydown', (e: KeyboardEvent) => {
            if (e.ctrlKey || e.metaKey) {
                // Check for +, -, =, and 0 
                if (e.key === '+' || e.key === '=' || e.key === '-' || e.key === '0') {
                    e.preventDefault();
                }
            }
        });

        //DISABLE IPAD DEFAULT GESTURE ZOOM
        document.addEventListener('gesturestart', (e: Event) => e.preventDefault(), { passive: false });
        document.addEventListener('gesturechange', (e: Event) => e.preventDefault(), { passive: false });
        document.addEventListener('gestureend', (e: Event) => e.preventDefault(), { passive: false });

        //DISABLE MULTI-TOUCH SWIPING, ALLOW THE CUSTOM CANVAS ZOOM
        document.addEventListener('touchmove', (e: TouchEvent) => {
            if (e.touches.length > 1) {
                e.preventDefault();
            }
        }, { passive: false });

        //Calculates clamp boundaries and applies the transform
        const updateTransform = () => {
            if (!this.ui.zoomWrapper) return;
            
            //Clamp Pan to prevent dragging the canvas entirely off-screen
            const rect = this.ui.viewport.getBoundingClientRect();
            const maxPanX = 0;
            const minPanX = rect.width - (rect.width * this.zoomLevel);
            const maxPanY = 0;
            const minPanY = rect.height - (rect.height * this.zoomLevel);

            //If zoomed all the way out, snap to center
            if (this.zoomLevel <= this.MIN_ZOOM) {
                this.panX = 0;
                this.panY = 0;
                this.zoomLevel = this.MIN_ZOOM;
            } else {
                this.panX = Math.min(Math.max(this.panX, minPanX), maxPanX);
                this.panY = Math.min(Math.max(this.panY, minPanY), maxPanY);
            }

            this.ui.zoomWrapper.style.transform = `translate(${this.panX}px, ${this.panY}px) scale(${this.zoomLevel})`;

            // --- Update UI Indicator ---
            if (this.ui.zoomIndicator && this.ui.zoomText) {
                if (this.zoomLevel > this.MIN_ZOOM) {
                    this.ui.zoomIndicator.classList.remove('hidden');
                    this.ui.zoomText.textContent = `${Math.round(this.zoomLevel * 100)}%`;
                } else {
                    this.ui.zoomIndicator.classList.add('hidden');
                }
            }
        };

        //Zoom in not just on center
        const applyZoomAt = (clientX: number, clientY: number, newZoom: number) => {
            const viewportRect = this.ui.viewport.getBoundingClientRect();

            //Convert screen coordinates to viewport-local coordinates
            const x = clientX - viewportRect.left;
            const y = clientY - viewportRect.top;

            //Find where that point currently lies on the unscaled/unpanned canvas
            const unscaledX = (x - this.panX) / this.zoomLevel;
            const unscaledY = (y - this.panY) / this.zoomLevel;

            this.zoomLevel = newZoom;

            //Recalculate panning so the unscaled point remains exactly under the cursor
            this.panX = x - (unscaledX * this.zoomLevel);
            this.panY = y - (unscaledY * this.zoomLevel);

            updateTransform();
        };

        // --- Mouse Wheel Zooming ---
        this.ui.viewport.addEventListener('wheel', (e: WheelEvent) => {
            if (!e.ctrlKey && !e.metaKey) return;
            e.preventDefault();

            const zoomSpeed = 0.003;
            const delta = -e.deltaY * zoomSpeed;
            const newZoom = Math.min(Math.max(this.zoomLevel * (1 + delta), this.MIN_ZOOM), this.MAX_ZOOM);

            applyZoomAt(e.clientX, e.clientY, newZoom);
        }, { passive: false });

        // --- Desktop Panning (Middle mouse or Alt left click) ---
        let isMousePanning = false;
        let lastMouseX = 0;
        let lastMouseY = 0;

        this.ui.viewport.addEventListener('pointerdown', (e: PointerEvent) => {
            //Button 1 is middle mouse. Or fallback to alt click for trackpads.
            if (e.button === 1 || (e.button === 0 && e.altKey)) {
                isMousePanning = true;
                lastMouseX = e.clientX;
                lastMouseY = e.clientY;
                e.preventDefault();
            }
        });

        window.addEventListener('pointermove', (e: PointerEvent) => {
            if (isMousePanning) {
                e.preventDefault();
                this.panX += e.clientX - lastMouseX;
                this.panY += e.clientY - lastMouseY;
                lastMouseX = e.clientX;
                lastMouseY = e.clientY;
                updateTransform();
            }
        });

        window.addEventListener('pointerup', () => { isMousePanning = false; });
        window.addEventListener('pointercancel', () => { isMousePanning = false; });

        // --- iPad Touch pinch to zoom and 2 finger pan ---
        let initialDistance = 0;
        let startZoom = 1;
        let lastCenter = { x: 0, y: 0 };

        const getDistance = (touches: TouchList) => {
            const dx = touches[0].clientX - touches[1].clientX;
            const dy = touches[0].clientY - touches[1].clientY;
            return Math.sqrt(dx * dx + dy * dy);
        };

        const getCenter = (touches: TouchList) => {
            return {
                x: (touches[0].clientX + touches[1].clientX) / 2,
                y: (touches[0].clientY + touches[1].clientY) / 2
            };
        };

        this.ui.viewport.addEventListener('touchstart', (e: TouchEvent) => {
            if (e.touches.length === 2) {
                e.preventDefault();
                initialDistance = getDistance(e.touches);
                startZoom = this.zoomLevel;
                lastCenter = getCenter(e.touches);
            }
        }, { passive: false });

        this.ui.viewport.addEventListener('touchmove', (e: TouchEvent) => {
            if (e.touches.length === 2 && this.ui.zoomWrapper) {
                e.preventDefault();

                const currentDistance = getDistance(e.touches);
                const currentCenter = getCenter(e.touches);

                //Handle the panning shift caused by two fingers dragging across the screen
                this.panX += currentCenter.x - lastCenter.x;
                this.panY += currentCenter.y - lastCenter.y;
                lastCenter = currentCenter;

                //Handle the scaling
                const scaleFactor = currentDistance / initialDistance;
                const newZoom = Math.min(Math.max(startZoom * scaleFactor, this.MIN_ZOOM), this.MAX_ZOOM);

                // Zoom exactly towards the center point of the two fingers
                applyZoomAt(currentCenter.x, currentCenter.y, newZoom);
            }
        }, { passive: false });
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

        if (state.isLaserFiring !== undefined) {
            // Toggles a class on both the viewport and the robot marker
            this.ui.viewport.classList.toggle('laser-firing', state.isLaserFiring);
        }

        if (state.current_height !== undefined && state.current_height !== null) {
            //If this is our first time hearing from the robot since refreshing
            this.isHardwareHeightSynced = true;

            this.ui.heightTrigger.disabled = false;

            if (!this.isChangingHeight) {
                const heightCm = state.current_height * 100;
                const displayVal = heightCm.toFixed(1);
                //Update the physical slider position
                this.ui.heightSlider.value = displayVal;

                //Update the text label next to the slider
                if (this.ui.heightDisplay) {
                    this.ui.heightDisplay.textContent = `${displayVal}`;
                }

            }
        }

        if (state.path_preview) {
            //Check for duration at the root level first (Your ideal schema)
            let duration = state.preview_duration;

            //Fallback: Check if it is nested inside path_preview (Robot backend schema)
            if (duration === undefined && state.path_preview.time !== undefined) {
                //Safely extract whether it's an array or a direct number 10.5
                duration = Array.isArray(state.path_preview.time)
                    ? state.path_preview.time[0]
                    : state.path_preview.time;
            }

            if (!duration) {
                console.log("Did not receive a duration from backend. Will use default duration.");
            }

            this.previewManager.handlePathFromWebSocket(state.path_preview, duration);
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