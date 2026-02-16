//frontend/src/core/UIRegistry.ts

/**
 * UIRegistry
 *
 * Centralized cache of all DOM elements used by the application.
 * Queried once at construction time to avoid repeated getElementById calls.
 */

export interface UIRegistry {
    // --- Core Viewport ---
    viewport: HTMLElement;
    video: HTMLVideoElement;
    canvas: HTMLCanvasElement;
    ctx: CanvasRenderingContext2D;

    // --- Modals & Overlays ---
    overlay: HTMLElement;
    settingsPopup: HTMLElement;
    preparePopup: HTMLElement;

    // --- Action Buttons ---
    settingsBtn: HTMLButtonElement;
    settingsCloseBtn: HTMLButtonElement;

    // Top panel button that opens the popup
    prepareBtn: HTMLButtonElement;

    // Popup buttons
    prepareCloseBtn: HTMLButtonElement;
    prepareCancelBtn: HTMLButtonElement;
    executeBtn: HTMLButtonElement;
    clearBtn: HTMLButtonElement;
    previewOverlay: HTMLCanvasElement;
    previewToggleOn: HTMLButtonElement;
    previewToggleOff: HTMLButtonElement;
    previewInfoPanel: HTMLElement;
    previewDuration: HTMLElement;
    previewMarker: HTMLElement;

    // --- Hardware Controls (Robot / Laser) ---
    robotBtn: HTMLButtonElement;
    laserBtn: HTMLButtonElement;

    // --- View & Mode Toggles ---
    processingModeSwitch: HTMLInputElement;
    transformedModeSwitch: HTMLInputElement;
    saveView: HTMLInputElement;
    layoutTopBtn: HTMLButtonElement;
    layoutBottomBtn: HTMLButtonElement;
    layoutLeftBtn: HTMLButtonElement;
    layoutRightBtn: HTMLButtonElement;

    // --- Input Fields ---
    speedInput: HTMLInputElement;

    // --- Element Groups (NodeLists) ---
    batchUiElements: NodeListOf<HTMLElement>;
    realTimeUIElements: NodeListOf<HTMLElement>;
    toggleButtons: NodeListOf<HTMLButtonElement>;
    sidebarButtons: NodeListOf<HTMLButtonElement>;
    settingsPanels: NodeListOf<HTMLElement>;

    // --- Visual Markers ---
    robotMarker: HTMLElement;

    // --- Shape Drawing Tools ---
    penBtn: HTMLButtonElement;
    realTimePen: HTMLButtonElement;
    squareBtn: HTMLButtonElement;
    circleBtn: HTMLButtonElement;
    triangleBtn: HTMLButtonElement;
    lineBtn: HTMLButtonElement;
    markerBtn: HTMLButtonElement;
    clearMarkersBtn: HTMLButtonElement;

    // --- Raster / Fill Settings ---
    fillOffBtn: HTMLButtonElement;
    fillOnBtn: HTMLButtonElement;
    fillSettingsPanel: HTMLElement;
    rasterBtnA: HTMLButtonElement;
    rasterBtnB: HTMLButtonElement;
    rasterDensityInput: HTMLInputElement;

    // --- Virtual Fixtures (Boundaries) ---
    fixturesTools: HTMLElement;
    drawingTools: HTMLElement;
    thermalTools: HTMLElement;
    roundBrushBtn: HTMLButtonElement;
    squareBrushBtn: HTMLButtonElement;
    brushSizeSlider: HTMLInputElement;
    clearBoundaryBtn: HTMLButtonElement;
    applyFixturesBtn: HTMLButtonElement;
    eraserBrushBtn: HTMLButtonElement;

    // Groups for toggling visibility based on mode
    fixturesUiElements: NodeListOf<HTMLElement>;
    drawingUiElements: NodeListOf<HTMLElement>;
    thermalUiElements: NodeListOf<HTMLElement>;

    // --- Thermal Data ---
    maxHeatDisplay: HTMLElement;
    heatAreaBtn: HTMLButtonElement;
    resetHeatAreaBtn: HTMLButtonElement;
    heatLegend: HTMLElement;

    // --- Main Mode Switchers ---
    modeButtons: NodeListOf<HTMLElement>;

    // Universal UI Elements
    heightSlider: HTMLInputElement;
    heightDisplay: HTMLElement;
}

export function createUIRegistry(): UIRegistry {
    return {
        // Core Viewport
        viewport: document.getElementById('viewport') as HTMLElement,
        video: document.getElementById('video') as HTMLVideoElement,
        canvas: document.getElementById('canvas') as HTMLCanvasElement,
        ctx: (document.getElementById('canvas') as HTMLCanvasElement).getContext('2d')!,

        // Modals & Overlays
        overlay: document.getElementById('overlay') as HTMLElement,
        settingsPopup: document.getElementById('settingsPopup') as HTMLElement,
        preparePopup: document.getElementById('preparePopup') as HTMLElement,

        // Action Buttons
        settingsBtn: document.getElementById('settingsBtn') as HTMLButtonElement,
        settingsCloseBtn: document.getElementById('settingsCloseBtn') as HTMLButtonElement,

        prepareBtn: document.getElementById('prepareBtn') as HTMLButtonElement,
        prepareCloseBtn: document.getElementById('prepareCloseBtn') as HTMLButtonElement,
        prepareCancelBtn: document.getElementById('prepareCancelBtn') as HTMLButtonElement,
        executeBtn: document.getElementById('executeBtn') as HTMLButtonElement,
        previewOverlay: document.getElementById('previewOverlay') as HTMLCanvasElement,
        previewToggleOn: document.getElementById('previewOn') as HTMLButtonElement,
        previewToggleOff: document.getElementById('previewOff') as HTMLButtonElement,
        previewInfoPanel: document.getElementById('previewInfoPanel') as HTMLElement,
        previewDuration: document.getElementById('previewDuration') as HTMLElement,
        previewMarker: document.getElementById('preview-marker') as HTMLElement,
        clearBtn: document.getElementById('clearBtn') as HTMLButtonElement,

        // Hardware Controls
        robotBtn: document.getElementById('robot-toggle-container') as HTMLButtonElement,
        laserBtn: document.getElementById('laser-toggle-container') as HTMLButtonElement,

        // View & Mode Toggles
        processingModeSwitch: document.getElementById('processing-mode') as HTMLInputElement,
        transformedModeSwitch: document.getElementById('transformed-view-mode') as HTMLInputElement,
        saveView: document.getElementById('save-view') as HTMLInputElement,
        layoutTopBtn: document.getElementById('layout-top') as HTMLButtonElement,
        layoutBottomBtn: document.getElementById('layout-bottom') as HTMLButtonElement,
        layoutLeftBtn: document.getElementById('layout-left') as HTMLButtonElement,
        layoutRightBtn: document.getElementById('layout-right') as HTMLButtonElement,

        // Input Fields
        speedInput: document.getElementById('speedInput') as HTMLInputElement,

        // Element Groups
        batchUiElements: document.querySelectorAll('.batch-ui'),
        realTimeUIElements: document.querySelectorAll('.real-time-ui'),
        toggleButtons: document.querySelectorAll('#middle-icon-section .icon-btn') as NodeListOf<HTMLButtonElement>,
        sidebarButtons: document.querySelectorAll('.settings-sidebar .sidebar-btn') as NodeListOf<HTMLButtonElement>,
        settingsPanels: document.querySelectorAll('.settings-main .settings-panel') as NodeListOf<HTMLElement>,

        // Visual Markers
        robotMarker: document.getElementById('robot-marker') as HTMLElement,

        // Shape Drawing Tools
        penBtn: document.getElementById('penBtn') as HTMLButtonElement,
        realTimePen: document.getElementById('realTimePen') as HTMLButtonElement,
        squareBtn: document.getElementById('squareBtn') as HTMLButtonElement,
        circleBtn: document.getElementById('circleBtn') as HTMLButtonElement,
        triangleBtn: document.getElementById('triangleBtn') as HTMLButtonElement,
        lineBtn: document.getElementById('lineBtn') as HTMLButtonElement,
        markerBtn: document.getElementById('markerBtn') as HTMLButtonElement,
        clearMarkersBtn: document.getElementById('clearMarkersBtn') as HTMLButtonElement,

        // Raster / Fill Settings
        fillOffBtn: document.getElementById('fillOff') as HTMLButtonElement,
        fillOnBtn: document.getElementById('fillOn') as HTMLButtonElement,
        fillSettingsPanel: document.getElementById('fillSettingsPanel') as HTMLElement,
        rasterBtnA: document.getElementById('rasterA') as HTMLButtonElement,
        rasterBtnB: document.getElementById('rasterB') as HTMLButtonElement,
        rasterDensityInput: document.getElementById('densityRaster') as HTMLInputElement,

        // Virtual Fixtures
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

        // Thermal Data
        maxHeatDisplay: document.getElementById('max-heat-display') as HTMLElement,
        heatAreaBtn: document.getElementById('heatAreaBtn') as HTMLButtonElement,
        resetHeatAreaBtn: document.getElementById('resetHeatAreaBtn') as HTMLButtonElement,
        heatLegend: document.getElementById('thermal-gradient-legend') as HTMLElement,

        // Main Mode Switchers
        modeButtons: document.querySelectorAll('.mode-btn'),

        // Universal UI Elements
        heightSlider: document.getElementById('heightSlider') as HTMLInputElement,
        heightDisplay: document.getElementById('heightDisplay') as HTMLElement,
    };
}