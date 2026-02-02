import { DrawingTracker } from './drawing/DrawingTracker';
import { ShapeType } from './drawing/types';
import { WebSocketHandler, WebSocketMessage } from './classes/WebSocketHandler';

declare global {
    interface Window {
        MediaMTXWebRTCReader: any;
    }
}

class VideoStreamController {
    // --- UI Elements ---
    private ui = {
        viewport: document.getElementById('viewport') as HTMLElement,
        video: document.getElementById('video') as HTMLVideoElement,
        canvas: document.getElementById('canvas') as HTMLCanvasElement,
        ctx: (document.getElementById('canvas') as HTMLCanvasElement).getContext("2d")!,
        
        // Modals & Overlays
        overlay: document.getElementById('overlay') as HTMLElement,
        settingsPopup: document.getElementById('settingsPopup') as HTMLElement,
        preparePopup: document.getElementById('preparePopup') as HTMLElement,
        
        // Buttons - Main
        settingsBtn: document.getElementById('settingsBtn') as HTMLButtonElement,
        settingsCloseBtn: document.getElementById('settingsCloseBtn') as HTMLButtonElement,
        prepareBtn: document.getElementById('prepareBtn') as HTMLButtonElement,
        prepareCloseBtn: document.getElementById('prepareCloseBtn') as HTMLButtonElement,
        prepareCancelBtn: document.getElementById('prepareCancelBtn') as HTMLButtonElement,
        executeBtn: document.getElementById('executeBtn') as HTMLButtonElement,
        clearBtn: document.getElementById('clearBtn') as HTMLButtonElement,
        
        // Toggles
        robotBtn: document.getElementById('robot-toggle-container') as HTMLButtonElement,
        laserBtn: document.getElementById('laser-toggle-container') as HTMLButtonElement,
        processingModeSwitch: document.getElementById('processing-mode') as HTMLInputElement,
        transformedModeSwitch: document.getElementById('transformed-view-mode') as HTMLInputElement,
        saveView: document.getElementById('save-view') as HTMLInputElement,
        
        // Inputs
        speedInput: document.getElementById('speedInput') as HTMLInputElement,
        
        // Groups
        batchUiElements: document.querySelectorAll('.batch-ui'),
        toggleButtons: document.querySelectorAll('#middle-icon-section .icon-btn') as NodeListOf<HTMLButtonElement>,
        sidebarButtons: document.querySelectorAll('.settings-sidebar .sidebar-btn') as NodeListOf<HTMLButtonElement>,
        settingsPanels: document.querySelectorAll('.settings-main .settings-panel') as NodeListOf<HTMLElement>,
        
        // Markers
        robotMarker: document.getElementById('robot-marker') as HTMLElement,
        
        // Shape Tools
        penBtn: document.getElementById('penBtn') as HTMLButtonElement,
        squareBtn: document.getElementById('squareBtn') as HTMLButtonElement,
        circleBtn: document.getElementById('circleBtn') as HTMLButtonElement,
        triangleBtn: document.getElementById('triangleBtn') as HTMLButtonElement,
        lineBtn: document.getElementById('lineBtn') as HTMLButtonElement,
        markerBtn: document.getElementById('markerBtn') as HTMLButtonElement,
        clearMarkersBtn: document.getElementById('clearMarkersBtn') as HTMLButtonElement,
        
        // Fill Settings
        fillAccordionToggle: document.getElementById('fillAccordionToggle') as HTMLButtonElement,
        fillSettingsPanel: document.getElementById('fillSettingsPanel') as HTMLElement,
        rasterBtnA: document.getElementById('rasterA') as HTMLButtonElement,
        rasterBtnB: document.getElementById('rasterB') as HTMLButtonElement,
        rasterDensityInput: document.getElementById('densityRaster') as HTMLInputElement,
        
        // Fixtures
        fixturesTools: document.getElementById('fixtures-tools') as HTMLElement,
        drawingTools: document.getElementById('drawing-tools') as HTMLElement,
        thermalTools: document.getElementById('thermal-tools') as HTMLElement,
        roundBrushBtn: document.getElementById('roundBrushBtn') as HTMLButtonElement,
        squareBrushBtn: document.getElementById('squareBrushBtn') as HTMLButtonElement,
        brushSizeSlider: document.getElementById('brushSizeSlider') as HTMLInputElement,
        clearBoundaryBtn: document.getElementById('clearBoundaryBtn') as HTMLButtonElement,
        applyFixturesBtn: document.getElementById('applyFixturesBtn') as HTMLButtonElement,
        eraserBrushBtn: document.getElementById('eraserBtn') as HTMLButtonElement,
        fixturesUiElements: document.querySelectorAll('.fixtures-ui-only'),
        drawingUiElements: document.querySelectorAll('.drawing-ui-only'),
        thermalUiElements: document.querySelectorAll('.thermal-ui'),
        
        // Heat
        averageHeatDisplay: document.getElementById('average-heat-display') as HTMLElement,
        heatAreaBtn: document.getElementById('heatAreaBtn') as HTMLButtonElement,
        resetHeatAreaBtn: document.getElementById('resetHeatAreaBtn') as HTMLButtonElement,
        
        // Mode Switching
        modeButtons: document.querySelectorAll('.mode-btn')
    };

    // --- State ---
    private state = {
        laserConfirmationTimeout: null as number | null,
        robotConfirmationTimeout: null as number | null,
        selectedShape: null as ShapeType | 'marker' | null,
        drawnShapeType: null as ShapeType | null,
        isRealTimeDrawing: false,
        latestRealTimePos: null as { x: number, y: number } | null,
        fillEnabled: false,
        selectedRasterPattern: 'line_raster' as 'line_raster' | 'spiral_raster' | null,
        currentMode: 'drawing' as 'drawing' | 'thermal' | 'fixtures',
        selectedBrushType: null as 'round' | 'square' | null,
        isEraserActive: false
    };

    private wsHandler: WebSocketHandler;
    private drawingTracker: DrawingTracker | null = null;
    private reader: any = null;

    constructor() {
        this.wsHandler = new WebSocketHandler(null);
        this.init();
    }

    private init() {
        this.setupVideoCanvas();
        this.setupWebSocket();
        this.bindEvents();
        this.setupInitialState();
    }

    private setupInitialState() {
        this.ui.video.muted = true;
        this.ui.video.autoplay = true;
        this.ui.canvas.width = this.ui.viewport.offsetWidth;
        this.ui.canvas.height = this.ui.viewport.offsetHeight;
        this.setMessage("Loading stream");
        document.getElementById('drawingBtn')?.classList.add('active');
    }

    private setMessage(str: string) {
        this.ui.ctx.fillText(str, 10, 50);
    }

    // --- WebSocket & Video ---

    private setupWebSocket() {
        this.wsHandler.onStateUpdate = (newState: WebSocketMessage) => this.syncUiToState(newState);
        this.wsHandler.connect();
    }

    private setupVideoCanvas() {
        this.reader = new window.MediaMTXWebRTCReader({
            url: new URL(`http://${window.location.hostname}:8889/mystream/whep`),
            onError: (err: string) => this.setMessage(err),
            onTrack: (evt: RTCTrackEvent) => {
                if (evt.track.kind === 'video') {
                    this.ui.video.srcObject = evt.streams[0];
                    this.ui.video.requestVideoFrameCallback(this.updateCanvasLoop.bind(this));
                    this.initDrawingTracker();
                }
            },
        });
    }

    private initDrawingTracker() {
        if (this.drawingTracker) this.drawingTracker.dispose();

        this.drawingTracker = new DrawingTracker(
            this.ui.canvas,
            this.ui.video,
            `http://${window.location.hostname}:443`,
            () => this.onShapeComplete(),
            () => this.updateFixturesButtonState(),
            () => this.updateThermalButtonState(),
            () => { this.ui.resetHeatAreaBtn.disabled = false; }
        );

        this.updateDrawButtonState();
        this.updateFixturesButtonState();
        this.updateThermalButtonState();
    }

    private updateCanvasLoop() {
        this.ui.ctx.drawImage(this.ui.video, 0, 0, this.ui.canvas.width, this.ui.canvas.height);
        
        if (this.ui.processingModeSwitch && !this.ui.processingModeSwitch.checked && this.drawingTracker) {
            this.drawingTracker.render();
        }
        this.ui.video.requestVideoFrameCallback(this.updateCanvasLoop.bind(this));
    }

    // --- Event Binding ---

    private bindEvents() {
        // Modals
        this.ui.settingsBtn.addEventListener('click', () => this.openSettings());
        this.ui.settingsCloseBtn.addEventListener('click', () => this.closeSettings());
        this.ui.prepareBtn.addEventListener('click', () => this.ui.preparePopup.classList.add('active'));
        this.ui.prepareCloseBtn.addEventListener('click', () => this.ui.preparePopup.classList.remove('active'));
        this.ui.prepareCancelBtn.addEventListener('click', () => this.ui.preparePopup.classList.remove('active'));
        this.ui.overlay.addEventListener('click', () => {
            if (this.ui.settingsPopup.classList.contains('active')) this.closeSettings();
            if (this.ui.preparePopup.classList.contains('active')) this.ui.preparePopup.classList.remove('active');
        });

        // Sidebar
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

        // Real-time Drawing
        this.ui.viewport.addEventListener('pointerdown', this.handleRealTimeStart.bind(this));
        this.ui.viewport.addEventListener('pointermove', (e) => {
            if (this.state.isRealTimeDrawing) {
                e.preventDefault();
                this.state.latestRealTimePos = this.getCanvasCoordinates(e.clientX, e.clientY);
            }
        });
        this.ui.viewport.addEventListener('pointerup', this.handleRealTimeEnd.bind(this));
        this.ui.viewport.addEventListener('pointercancel', this.handleRealTimeEnd.bind(this));

        // Controls
        this.ui.processingModeSwitch.addEventListener('change', () => this.toggleMode());
        this.ui.modeButtons.forEach(btn => {
            btn.addEventListener('click', () => {
                this.ui.modeButtons.forEach(b => b.classList.remove('active'));
                btn.classList.add('active');
                this.switchMode(btn.id);
            });
        });

        // Tools
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

        // Brushes
        this.ui.roundBrushBtn.addEventListener('click', () => this.handleBrushSelection('round'));
        this.ui.squareBrushBtn.addEventListener('click', () => this.handleBrushSelection('square'));
        this.ui.eraserBrushBtn.addEventListener('click', () => this.handleEraserSelection());
        this.ui.brushSizeSlider.addEventListener('input', () => {
            if (this.state.selectedBrushType && this.drawingTracker) {
                this.drawingTracker.setFixturesBrush(this.state.selectedBrushType, parseInt(this.ui.brushSizeSlider.value), this.state.isEraserActive);
            }
        });

        // Fixtures Actions
        this.ui.clearBoundaryBtn.addEventListener('click', () => this.clearFixtures());
        this.ui.applyFixturesBtn.addEventListener('click', () => this.applyFixtures());

        // Robot/Laser
        this.ui.laserBtn.addEventListener('click', () => {
            this.ui.laserBtn.style.pointerEvents = 'none';
            if (this.state.laserConfirmationTimeout) clearTimeout(this.state.laserConfirmationTimeout);
            this.changeLaserState(!this.ui.laserBtn.classList.contains('active'));
        });
        this.ui.robotBtn.addEventListener('click', () => {
            this.ui.robotBtn.style.pointerEvents = 'none';
            if (this.state.robotConfirmationTimeout) clearTimeout(this.state.robotConfirmationTimeout);
            this.changeRobotState(!this.ui.robotBtn.classList.contains('active'));
        });

        // Shapes
        this.ui.penBtn.addEventListener('click', () => this.handleShapeSelection(this.ui.penBtn, 'freehand'));
        this.ui.squareBtn.addEventListener('click', () => this.handleShapeSelection(this.ui.squareBtn, 'square'));
        this.ui.circleBtn.addEventListener('click', () => this.handleShapeSelection(this.ui.circleBtn, 'circle'));
        this.ui.triangleBtn.addEventListener('click', () => this.handleShapeSelection(this.ui.triangleBtn, 'triangle'));
        this.ui.lineBtn.addEventListener('click', () => this.handleShapeSelection(this.ui.lineBtn, 'line'));
        
        this.ui.canvas.addEventListener('mouseup', () => setTimeout(() => this.updateDrawButtonState(), 50));
        this.ui.canvas.addEventListener('touchend', () => setTimeout(() => this.updateDrawButtonState(), 50));

        // Execution
        this.ui.executeBtn.addEventListener('click', () => this.executePath());
        this.ui.clearBtn.addEventListener('click', () => this.clearDrawing());
        this.ui.clearMarkersBtn.addEventListener('click', async () => {
            if (this.drawingTracker) {
                this.drawingTracker.clearMarkers();
                await this.drawingTracker.submitHeatMarkers(this.drawingTracker.getHeatMarkersInVideoSpace());
            }
        });

        // Fill Settings
        this.ui.fillAccordionToggle.addEventListener('click', () => {
            this.ui.fillSettingsPanel.classList.toggle('open');
            this.ui.fillAccordionToggle.classList.toggle('active');
            this.state.fillEnabled = this.ui.fillAccordionToggle.classList.contains('active');
        });
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

        // View
        this.ui.saveView.addEventListener('click', async () => {
            if (this.drawingTracker) await this.drawingTracker.updateViewSettings(this.ui.transformedModeSwitch.checked, false);
        });
        window.addEventListener('resize', () => {
            if (this.drawingTracker) this.drawingTracker.updateCanvasSize(this.ui.viewport.offsetWidth, this.ui.viewport.offsetHeight);
        });
    }

    // --- Logic Methods ---

    private syncUiToState(state: Partial<WebSocketMessage>) {
        if (state.x !== undefined && state.y !== undefined) {
            const containerWidth = this.ui.viewport.offsetWidth;
            const containerHeight = this.ui.viewport.offsetHeight;
            const videoWidth = this.ui.video.videoWidth;
            const videoHeight = this.ui.video.videoHeight;

            if (videoWidth > 0 && videoHeight > 0) {
                const cssLeft = (state.x / videoWidth) * containerWidth;
                const cssTop = (state.y / videoHeight) * containerHeight;
                this.ui.robotMarker.style.left = `${cssLeft}px`;
                this.ui.robotMarker.style.top = `${cssTop}px`;
                this.ui.robotMarker.style.display = 'block';
            }
        }
        if (state.averageHeat !== undefined) this.updateAverageHeat(state.averageHeat);
        if (state.heat_markers && this.drawingTracker) this.drawingTracker.updateMarkerTemperatures(state.heat_markers);
        if (state.isLaserOn !== undefined) this.syncToggleState(this.ui.laserBtn, !!state.isLaserOn, this.state.laserConfirmationTimeout);
        if (state.isRobotOn !== undefined) this.syncToggleState(this.ui.robotBtn, !!state.isRobotOn, this.state.robotConfirmationTimeout);
    }

    private syncToggleState(btn: HTMLElement, incomingState: boolean, timeoutRef: number | null) {
        if (timeoutRef) {
            if (incomingState === btn.classList.contains('active')) btn.style.pointerEvents = 'auto';
        } else {
            btn.classList.toggle('active', incomingState);
            btn.style.pointerEvents = 'auto';
        }
    }

    private updateAverageHeat(heat: number | null | undefined) {
        if (!this.ui.averageHeatDisplay) return;
        this.ui.averageHeatDisplay.textContent = (heat === null || heat === undefined || isNaN(heat)) ? 'N/A' : `${heat.toFixed(1)}°C`;
    }

    private getCanvasCoordinates(clientX: number, clientY: number) {
        const rect = this.ui.canvas.getBoundingClientRect();
        return { x: clientX - rect.left, y: clientY - rect.top };
    }

    private handleRealTimeStart(e: PointerEvent) {
        if (!this.ui.processingModeSwitch.checked || this.state.selectedShape !== 'freehand') return;
        e.preventDefault();
        this.ui.canvas.setPointerCapture(e.pointerId);
        this.state.isRealTimeDrawing = true;
        this.state.latestRealTimePos = this.getCanvasCoordinates(e.clientX, e.clientY);
        this.wsHandler.updateState({ pathEvent: 'start' });
        this.runRealTimeLoop();
    }

    private runRealTimeLoop() {
        if (!this.state.isRealTimeDrawing) return;
        if (this.state.latestRealTimePos) {
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

    private toggleMode() {
        const isRealTime = this.ui.processingModeSwitch.checked;
        if (isRealTime) {
            this.ui.batchUiElements.forEach(el => el.classList.add('hidden-mode'));
            this.drawingTracker?.disableDrawing();
        } else {
            this.ui.batchUiElements.forEach(el => el.classList.remove('hidden-mode'));
            this.drawingTracker?.disableDrawing();
        }
        this.ui.toggleButtons.forEach(btn => {
            btn.classList.remove('selected');
            btn.disabled = false;
        });
        this.state.selectedShape = null;
        this.state.drawnShapeType = null;
        this.updateDrawButtonState();
    }

    private switchMode(modeId: string) {
        console.log("Switching to mode:", modeId);
        
        // Cleanup previous mode
        if (this.state.currentMode === 'fixtures' && modeId !== 'fixturesBtn') {
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

        // Activate new mode
        if (modeId === 'fixturesBtn') {
            this.state.currentMode = 'fixtures';
            this.ui.drawingTools.classList.add('hidden');
            this.ui.thermalTools.classList.add('hidden');
            this.ui.fixturesTools.classList.remove('hidden');
            this.ui.thermalUiElements.forEach(el => el.classList.add('hidden'));
            this.ui.drawingUiElements.forEach(el => el.classList.add('hidden'));
            this.ui.fixturesUiElements.forEach(el => el.classList.remove('hidden'));
            this.drawingTracker?.disableDrawing();
            this.drawingTracker?.enableFixturesMode();
            this.updateFixturesButtonState();
        } else if (modeId === 'thermalBtn') {
            this.state.currentMode = 'thermal';
            this.drawingTracker?.showMarkers();
            this.updateThermalButtonState();
            this.ui.drawingTools.classList.add('hidden');
            this.ui.thermalTools.classList.remove('hidden');
            this.ui.fixturesTools.classList.add('hidden');
            this.ui.drawingUiElements.forEach(el => el.classList.add('hidden'));
            this.ui.fixturesUiElements.forEach(el => el.classList.add('hidden'));
            this.drawingTracker?.disableFixturesMode();
            this.drawingTracker?.disableDrawing();
            if (this.drawingTracker?.hasFixtures()) this.drawingTracker?.showFixtures();
            this.ui.thermalUiElements.forEach(el => el.classList.remove('hidden'));
            if (this.state.selectedShape === 'marker') {
                this.ui.markerBtn.classList.add('selected');
                this.drawingTracker?.enableMarkerMode();
            }
        } else {
            this.state.currentMode = 'drawing';
            this.ui.drawingTools.classList.remove('hidden');
            this.ui.thermalTools.classList.add('hidden');
            this.ui.fixturesTools.classList.add('hidden');
            this.ui.heatAreaBtn.classList.remove('selected');
            this.ui.markerBtn.classList.remove('selected');
            this.drawingTracker?.disableMarkerMode();
            this.drawingTracker?.disableHeatAreaMode();
            this.ui.drawingUiElements.forEach(el => el.classList.remove('hidden'));
            this.ui.fixturesUiElements.forEach(el => el.classList.add('hidden'));
            this.ui.thermalUiElements.forEach(el => el.classList.add('hidden'));
            this.drawingTracker?.disableFixturesMode();
            if (this.drawingTracker?.hasFixtures()) this.drawingTracker?.showFixtures();
            
            if (this.state.drawnShapeType) {
                this.drawingTracker?.disableDrawing();
                this.ui.toggleButtons.forEach(btn => btn.classList.remove('selected'));
                this.highlightShapeBtn(this.state.drawnShapeType);
            } else if (this.state.selectedShape && this.state.selectedShape !== 'marker') {
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

    private highlightShapeBtn(shape: ShapeType) {
        if (shape === 'freehand') this.ui.penBtn.classList.add('selected');
        else if (shape === 'square') this.ui.squareBtn.classList.add('selected');
        else if (shape === 'circle') this.ui.circleBtn.classList.add('selected');
        else if (shape === 'triangle') this.ui.triangleBtn.classList.add('selected');
        else if (shape === 'line') this.ui.lineBtn.classList.add('selected');
    }

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

        if (this.state.selectedBrushType === brushType) {
            this.ui.roundBrushBtn.classList.remove('selected');
            this.ui.squareBrushBtn.classList.remove('selected');
            this.state.selectedBrushType = null;
            this.state.isEraserActive = false;
            this.drawingTracker?.disableFixturesBrush();
        } else {
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
            this.ui.eraserBrushBtn.classList.remove('selected');
            this.state.selectedBrushType = null;
            this.state.isEraserActive = false;
            this.drawingTracker?.disableFixturesBrush();
        } else {
            this.ui.roundBrushBtn.classList.remove('selected');
            this.ui.squareBrushBtn.classList.remove('selected');
            this.ui.eraserBrushBtn.classList.add('selected');
            this.state.selectedBrushType = 'round';
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
            await this.drawingTracker.executeFixtures();
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

    private changeLaserState(newState: boolean) {
        this.ui.laserBtn.style.pointerEvents = 'none';
        const updates: any = { isLaserOn: newState };
        if (newState === false && this.ui.robotBtn.classList.contains('active')) {
            updates.isRobotOn = false;
            this.ui.robotBtn.style.pointerEvents = 'none';
            if (this.state.robotConfirmationTimeout) clearTimeout(this.state.robotConfirmationTimeout);
            this.state.robotConfirmationTimeout = setTimeout(() => { this.ui.robotBtn.style.pointerEvents = 'auto'; }, 2000);
        }
        this.wsHandler.updateState(updates);
        if (this.state.laserConfirmationTimeout) clearTimeout(this.state.laserConfirmationTimeout);
        this.state.laserConfirmationTimeout = setTimeout(() => { this.ui.laserBtn.style.pointerEvents = 'auto'; }, 2000);
    }

    private changeRobotState(newState: boolean) {
        this.ui.robotBtn.style.pointerEvents = 'none';
        const updates: any = { isRobotOn: newState };
        if (newState === false && this.ui.laserBtn.classList.contains('active')) {
            updates.isLaserOn = false;
            this.ui.laserBtn.style.pointerEvents = 'none';
            if (this.state.laserConfirmationTimeout) clearTimeout(this.state.laserConfirmationTimeout);
            this.state.laserConfirmationTimeout = setTimeout(() => { this.ui.laserBtn.style.pointerEvents = 'auto'; }, 2000);
        }
        this.wsHandler.updateState(updates);
        if (this.state.robotConfirmationTimeout) clearTimeout(this.state.robotConfirmationTimeout);
        this.state.robotConfirmationTimeout = setTimeout(() => { this.ui.robotBtn.style.pointerEvents = 'auto'; }, 2000);
    }

    private handleShapeSelection(button: HTMLButtonElement, shape: ShapeType | 'marker') {
        if (this.ui.processingModeSwitch.checked && shape !== 'freehand') return;
        if (button.disabled) return;

        if (button.classList.contains('selected')) {
            button.classList.remove('selected');
            this.state.selectedShape = null;
            this.drawingTracker?.disableDrawing();
            this.drawingTracker?.disableMarkerMode();
        } else {
            this.ui.toggleButtons.forEach(btn => btn.classList.remove('selected'));
            button.classList.add('selected');
            this.state.selectedShape = shape;

            if (this.ui.processingModeSwitch.checked) {
                this.drawingTracker?.clearDrawing();
                this.state.drawnShapeType = null;
            } else {
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
            await this.drawingTracker.executePath(speed, String(this.state.selectedRasterPattern), density, this.state.fillEnabled);
            this.drawingTracker.clearDrawing();
            this.state.drawnShapeType = null;
            this.ui.toggleButtons.forEach(btn => btn.disabled = false);
            this.updateDrawButtonState();
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
        if (this.drawingTracker && this.state.selectedShape && this.state.selectedShape !== 'marker') {
            this.drawingTracker.setShapeType(this.state.selectedShape);
            this.drawingTracker.enableDrawing();
        }
    }

    private onShapeComplete() {
        if (this.state.selectedShape && this.state.selectedShape !== 'marker') {
            this.state.drawnShapeType = this.state.selectedShape;
            this.updateDrawButtonState();
        }
    }

    private openSettings() {
        if (this.drawingTracker && this.drawingTracker.hasShape()) {
            this.drawingTracker.clearDrawing();
            this.state.selectedShape = null;
            this.state.drawnShapeType = null;
            this.ui.toggleButtons.forEach(btn => btn.classList.remove('selected'));
            this.updateDrawButtonState();
        }
        [this.ui.clearBtn, this.ui.prepareBtn, this.ui.robotBtn, this.ui.laserBtn].forEach(btn => btn.disabled = true);
        this.changeLaserState(false);
        this.changeRobotState(false);
        this.ui.settingsPopup.classList.add('active');
        this.ui.overlay.classList.add('active');
    }

    private closeSettings() {
        this.ui.settingsPopup.classList.remove('active');
        this.ui.overlay.classList.remove('active');
        [this.ui.clearBtn, this.ui.robotBtn, this.ui.laserBtn].forEach(btn => btn.disabled = false);
    }

    private updateDrawButtonState() {
        const hasShape = this.state.drawnShapeType !== null;
        this.ui.clearBtn.disabled = !hasShape;
        this.ui.prepareBtn.disabled = !hasShape;
        this.ui.executeBtn.disabled = !hasShape;

        if (hasShape) {
            this.ui.penBtn.disabled = (this.state.drawnShapeType !== 'freehand');
            this.ui.squareBtn.disabled = (this.state.drawnShapeType !== 'square');
            this.ui.circleBtn.disabled = (this.state.drawnShapeType !== 'circle');
            this.ui.triangleBtn.disabled = (this.state.drawnShapeType !== 'triangle');
            this.ui.lineBtn.disabled = (this.state.drawnShapeType !== 'line');
            this.ui.markerBtn.disabled = true;
        } else {
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
}

window.addEventListener('load', () => {
    new VideoStreamController();
});