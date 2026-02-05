// tests/ToolHandler.test.ts
import { describe, it, expect, beforeEach, vi } from 'vitest';
import { ToolHandler } from '../src/features/drawing/ToolHandler';
import { createAppState, AppState } from '../src/core/AppState';

// ---------------------------------------------------------------------------
// Helpers – fabricate the slice of UIRegistry that ToolHandler actually reads
// ---------------------------------------------------------------------------
function makeBtn(): HTMLButtonElement {
    const b = document.createElement('button');
    (b as any).disabled = false;
    return b;
}

function makeUI() {
    const toggleButtons = [makeBtn(), makeBtn(), makeBtn(), makeBtn(), makeBtn()];
    return {
        penBtn:          toggleButtons[0],
        squareBtn:       toggleButtons[1],
        circleBtn:       toggleButtons[2],
        triangleBtn:     toggleButtons[3],
        lineBtn:         toggleButtons[4],
        markerBtn:       makeBtn(),
        heatAreaBtn:     makeBtn(),
        clearMarkersBtn: makeBtn(),

        roundBrushBtn:   makeBtn(),
        squareBrushBtn:  makeBtn(),
        eraserBrushBtn:  makeBtn(),
        brushSizeSlider: { value: '60' } as any,

        clearBoundaryBtn: makeBtn(),
        applyFixturesBtn: makeBtn(),

        clearBtn:   { disabled: false },
        prepareBtn: { disabled: false },
        executeBtn: { disabled: false },

        toggleButtons,                        // mirrors the NodeList iteration pattern
        processingModeSwitch: { checked: false } as any,
    } as any;
}

function makeCM() {
    return {
        setShapeType:          vi.fn(),
        enableDrawing:         vi.fn(),
        clearDrawing: vi.fn(),
        disableDrawing:        vi.fn(),
        enableMarkerMode:      vi.fn(),
        disableMarkerMode:     vi.fn(),
        enableHeatAreaMode:    vi.fn(),
        disableHeatAreaMode:   vi.fn(),
        setFixturesBrush:      vi.fn(),
        disableFixturesBrush:  vi.fn(),
        clearFixtures:         vi.fn(),
        clearFixturesOnServer: vi.fn().mockResolvedValue({}),
        executeFixtures:       vi.fn().mockResolvedValue({}),
        hasFixtures:           vi.fn(() => true),
        canApplyFixtures:      vi.fn(() => true),
        hasMarkers:            vi.fn(() => false),
    } as any;
}

// ---------------------------------------------------------------------------
describe('ToolHandler – handleShapeSelection (batch mode)', () => {
    let ui: any, state: AppState, cm: any, th: ToolHandler;

    beforeEach(() => {
        ui    = makeUI();
        state = createAppState();
        cm    = makeCM();
        th    = new ToolHandler(ui, state, () => cm);
    });

    it('selecting a shape sets state and calls setShapeType', () => {
        th.handleShapeSelection(ui.squareBtn, 'square');

        expect(state.selectedShape).toBe('square');
        expect(ui.squareBtn.classList.contains('selected')).toBe(true);
        expect(cm.setShapeType).toHaveBeenCalledWith('square');
        expect(cm.enableDrawing).toHaveBeenCalled();
    });

    it('clicking the same button again deselects it', () => {
        // First click → select
        th.handleShapeSelection(ui.circleBtn, 'circle');
        // Second click → deselect
        th.handleShapeSelection(ui.circleBtn, 'circle');

        expect(state.selectedShape).toBeNull();
        expect(ui.circleBtn.classList.contains('selected')).toBe(false);
        expect(cm.disableDrawing).toHaveBeenCalled();
    });

    it('selecting a new shape removes "selected" from all others', () => {
        th.handleShapeSelection(ui.penBtn, 'freehand');
        th.handleShapeSelection(ui.lineBtn, 'line');

        expect(ui.penBtn.classList.contains('selected')).toBe(false);
        expect(ui.lineBtn.classList.contains('selected')).toBe(true);
    });

    it('does nothing when the button is disabled', () => {
        (ui.squareBtn as any).disabled = true;
        th.handleShapeSelection(ui.squareBtn, 'square');
        expect(state.selectedShape).toBeNull();
    });

    it('marker shape activates marker mode', () => {
        th.handleShapeSelection(ui.markerBtn as any, 'marker');
        expect(cm.enableMarkerMode).toHaveBeenCalled();
        expect(state.selectedShape).toBe('marker');
    });
});

// ---------------------------------------------------------------------------
describe('ToolHandler – handleShapeSelection (real-time mode)', () => {
    let ui: any, state: AppState, cm: any, th: ToolHandler;

    beforeEach(() => {
        ui    = makeUI();
        ui.processingModeSwitch.checked = true;   // real-time ON
        state = createAppState();
        cm    = makeCM();
        th    = new ToolHandler(ui, state, () => cm);
    });

    it('only freehand is allowed in real-time mode', () => {
        th.handleShapeSelection(ui.squareBtn, 'square');
        expect(state.selectedShape).toBeNull();   // blocked
        expect(cm.setShapeType).not.toHaveBeenCalled();
    });

    it('freehand is allowed and clears any previous drawing', () => {
        state.drawnShapeType = 'circle';   // pretend something was drawn
        th.handleShapeSelection(ui.penBtn, 'freehand');

        expect(state.selectedShape).toBe('freehand');
        expect(cm.clearDrawing).toHaveBeenCalled();
        expect(state.drawnShapeType).toBeNull();
    });
});

// ---------------------------------------------------------------------------
describe('ToolHandler – selectThermalTool', () => {
    let ui: any, state: AppState, cm: any, th: ToolHandler;

    beforeEach(() => {
        ui    = makeUI();
        state = createAppState();
        cm    = makeCM();
        th    = new ToolHandler(ui, state, () => cm);
    });

    it('activating marker enables marker mode on CM', () => {
        th.selectThermalTool('marker');
        expect(cm.enableMarkerMode).toHaveBeenCalled();
        expect(ui.markerBtn.classList.contains('selected')).toBe(true);
        expect(state.selectedShape).toBe('marker');
    });

    it('activating heat enables heat-area mode on CM', () => {
        th.selectThermalTool('heat');
        expect(cm.enableHeatAreaMode).toHaveBeenCalled();
        expect(ui.heatAreaBtn.classList.contains('selected')).toBe(true);
    });

    it('clicking marker again toggles it OFF', () => {
        th.selectThermalTool('marker');    // ON
        th.selectThermalTool('marker');    // OFF

        expect(ui.markerBtn.classList.contains('selected')).toBe(false);
        expect(cm.disableMarkerMode).toHaveBeenCalled();
        expect(state.selectedShape).toBeNull();
    });

    it('clicking heat again toggles it OFF', () => {
        th.selectThermalTool('heat');
        th.selectThermalTool('heat');

        expect(ui.heatAreaBtn.classList.contains('selected')).toBe(false);
        expect(cm.disableHeatAreaMode).toHaveBeenCalled();
    });

    it('switching from marker → heat disables marker first', () => {
        th.selectThermalTool('marker');
        th.selectThermalTool('heat');

        expect(ui.markerBtn.classList.contains('selected')).toBe(false);
        expect(ui.heatAreaBtn.classList.contains('selected')).toBe(true);
        expect(cm.disableMarkerMode).toHaveBeenCalled();
        expect(cm.enableHeatAreaMode).toHaveBeenCalled();
    });
});

// ---------------------------------------------------------------------------
describe('ToolHandler – fixture brush lifecycle', () => {
    let ui: any, state: AppState, cm: any, th: ToolHandler;

    beforeEach(() => {
        ui    = makeUI();
        state = createAppState();
        cm    = makeCM();
        th    = new ToolHandler(ui, state, () => cm);
    });

    it('selecting round brush sets state and calls setFixturesBrush', () => {
        th.handleBrushSelection('round');
        expect(state.selectedBrushType).toBe('round');
        expect(state.isEraserActive).toBe(false);
        expect(cm.setFixturesBrush).toHaveBeenCalledWith('round', 60, false);
        expect(ui.roundBrushBtn.classList.contains('selected')).toBe(true);
    });

    it('selecting square brush works symmetrically', () => {
        th.handleBrushSelection('square');
        expect(state.selectedBrushType).toBe('square');
        expect(cm.setFixturesBrush).toHaveBeenCalledWith('square', 60, false);
    });

    it('clicking the same brush toggles it off', () => {
        th.handleBrushSelection('round');
        th.handleBrushSelection('round');   // toggle off

        expect(state.selectedBrushType).toBeNull();
        expect(cm.disableFixturesBrush).toHaveBeenCalled();
        expect(ui.roundBrushBtn.classList.contains('selected')).toBe(false);
    });

    it('switching from round → square clears round selection', () => {
        th.handleBrushSelection('round');
        th.handleBrushSelection('square');

        expect(ui.roundBrushBtn.classList.contains('selected')).toBe(false);
        expect(ui.squareBrushBtn.classList.contains('selected')).toBe(true);
    });

    it('does nothing when the target brush button is disabled', () => {
        (ui.roundBrushBtn as any).disabled = true;
        th.handleBrushSelection('round');
        expect(state.selectedBrushType).toBeNull();
    });
});

// ---------------------------------------------------------------------------
describe('ToolHandler – eraser', () => {
    let ui: any, state: AppState, cm: any, th: ToolHandler;

    beforeEach(() => {
        ui    = makeUI();
        state = createAppState();
        cm    = makeCM();
        th    = new ToolHandler(ui, state, () => cm);
    });

    it('activating eraser sets isEraserActive and calls setFixturesBrush with erase=true', () => {
        th.handleEraserSelection();
        expect(state.isEraserActive).toBe(true);
        expect(state.selectedBrushType).toBe('round');   // eraser is internally a round brush
        expect(cm.setFixturesBrush).toHaveBeenCalledWith('round', 60, true);
        expect(ui.eraserBrushBtn.classList.contains('selected')).toBe(true);
    });

    it('clicking eraser again toggles it off', () => {
        th.handleEraserSelection();
        th.handleEraserSelection();

        expect(state.isEraserActive).toBe(false);
        expect(state.selectedBrushType).toBeNull();
        expect(cm.disableFixturesBrush).toHaveBeenCalled();
    });

    it('activating eraser removes round/square selections', () => {
        th.handleBrushSelection('round');
        th.handleEraserSelection();

        expect(ui.roundBrushBtn.classList.contains('selected')).toBe(false);
        expect(ui.eraserBrushBtn.classList.contains('selected')).toBe(true);
    });

    it('does nothing when eraser button is disabled', () => {
        (ui.eraserBrushBtn as any).disabled = true;
        th.handleEraserSelection();
        expect(state.isEraserActive).toBe(false);
    });
});

// ---------------------------------------------------------------------------
describe('ToolHandler – handleBrushSizeChange', () => {
    let ui: any, state: AppState, cm: any, th: ToolHandler;

    beforeEach(() => {
        ui    = makeUI();
        state = createAppState();
        cm    = makeCM();
        th    = new ToolHandler(ui, state, () => cm);
    });

    it('forwards new size to CM when a brush is active', () => {
        state.selectedBrushType = 'square';
        ui.brushSizeSlider.value = '80';
        th.handleBrushSizeChange();
        expect(cm.setFixturesBrush).toHaveBeenCalledWith('square', 80, false);
    });

    it('is a no-op when no brush is active', () => {
        state.selectedBrushType = null;
        th.handleBrushSizeChange();
        expect(cm.setFixturesBrush).not.toHaveBeenCalled();
    });

    it('passes isEraserActive flag through', () => {
        state.selectedBrushType = 'round';
        state.isEraserActive    = true;
        ui.brushSizeSlider.value = '40';
        th.handleBrushSizeChange();
        expect(cm.setFixturesBrush).toHaveBeenCalledWith('round', 40, true);
    });
});

// ---------------------------------------------------------------------------
describe('ToolHandler – clearFixtures / applyFixtures', () => {
    let ui: any, state: AppState, cm: any, th: ToolHandler;

    beforeEach(() => {
        ui    = makeUI();
        state = createAppState();
        state.selectedBrushType = 'round';
        cm    = makeCM();
        th    = new ToolHandler(ui, state, () => cm);
    });

    it('clearFixtures calls cm.clearFixtures + server, resets brush state', async () => {
        await th.clearFixtures();

        expect(cm.clearFixtures).toHaveBeenCalled();
        expect(cm.clearFixturesOnServer).toHaveBeenCalled();
        expect(state.selectedBrushType).toBeNull();
        expect(state.isEraserActive).toBe(false);
        expect(cm.disableFixturesBrush).toHaveBeenCalled();
    });

    it('applyFixtures calls cm.executeFixtures and resets brush state', async () => {
        await th.applyFixtures();

        expect(cm.executeFixtures).toHaveBeenCalled();
        expect(state.selectedBrushType).toBeNull();
        expect(cm.disableFixturesBrush).toHaveBeenCalled();
    });

    it('applyFixtures re-enables buttons on failure', async () => {
        cm.executeFixtures = vi.fn().mockRejectedValue(new Error('fail'));
        await th.applyFixtures();

        expect((ui.applyFixturesBtn as any).disabled).toBe(false);
        expect((ui.clearBoundaryBtn as any).disabled).toBe(false);
    });

    it('clearFixtures is safe when cm is null', async () => {
        const thNull = new ToolHandler(ui, state, () => null);
        await expect(thNull.clearFixtures()).resolves.toBeUndefined();
    });
});

// ---------------------------------------------------------------------------
describe('ToolHandler – button-state updaters', () => {
    let ui: any, state: AppState, cm: any, th: ToolHandler;

    beforeEach(() => {
        ui    = makeUI();
        state = createAppState();
        cm    = makeCM();
        th    = new ToolHandler(ui, state, () => cm);
    });

    // --- updateDrawButtonState ---
    it('when drawnShapeType is null: clear/prepare/execute are disabled, all tools enabled', () => {
        state.drawnShapeType = null;
        th.updateDrawButtonState();

        expect((ui.clearBtn as any)?.disabled ?? (ui as any).clearBtn?.disabled).toBeTruthy();
        // All toggle buttons enabled
        ui.toggleButtons.forEach((btn: any) => expect(btn.disabled).toBe(false));
    });

    it('when drawnShapeType is set: only matching tool stays enabled', () => {
        state.drawnShapeType = 'circle';
        th.updateDrawButtonState();

        expect((ui.circleBtn as any).disabled).toBe(false);   // the drawn one
        expect((ui.penBtn as any).disabled).toBe(true);
        expect((ui.squareBtn as any).disabled).toBe(true);
        expect((ui.triangleBtn as any).disabled).toBe(true);
        expect((ui.lineBtn as any).disabled).toBe(true);
    });

    // --- updateFixturesButtonState ---
    it('enables clear when fixtures exist, enables apply when canApply', () => {
        cm.hasFixtures.mockReturnValue(true);
        cm.canApplyFixtures.mockReturnValue(true);
        th.updateFixturesButtonState();

        expect((ui.clearBoundaryBtn as any).disabled).toBe(false);
        expect((ui.applyFixturesBtn as any).disabled).toBe(false);
        expect((ui.eraserBrushBtn as any).disabled).toBe(false);   // has fixtures
    });

    it('disables clear & apply when no fixtures', () => {
        cm.hasFixtures.mockReturnValue(false);
        cm.canApplyFixtures.mockReturnValue(false);
        th.updateFixturesButtonState();

        expect((ui.clearBoundaryBtn as any).disabled).toBe(true);
        expect((ui.applyFixturesBtn as any).disabled).toBe(true);
        expect((ui.eraserBrushBtn as any).disabled).toBe(true);
    });

    // --- updateThermalButtonState ---
    it('enables clearMarkers when markers exist', () => {
        cm.hasMarkers.mockReturnValue(true);
        th.updateThermalButtonState();
        expect((ui.clearMarkersBtn as any).disabled).toBe(false);
    });

    it('disables clearMarkers when no markers', () => {
        cm.hasMarkers.mockReturnValue(false);
        th.updateThermalButtonState();
        expect((ui.clearMarkersBtn as any).disabled).toBe(true);
    });

    it('updateThermalButtonState is safe when CM is null', () => {
        const thNull = new ToolHandler(ui, state, () => null);
        expect(() => thNull.updateThermalButtonState()).not.toThrow();
    });
});
