// tests/ModeManager.test.ts
import { describe, it, expect, beforeEach, vi, type Mock } from 'vitest';
import { ModeManager } from '../src/features/settings/ModeManager';
import { createAppState, AppState } from '../src/core/AppState';

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
function makeClassList(...initial: string[]) {
    const set = new Set(initial);
    return {
        add:      (c: string)  => set.add(c),
        remove:   (c: string)  => set.delete(c),
        contains: (c: string)  => set.has(c),
        toggle:   (c: string)  => set.has(c) ? set.delete(c) : set.add(c),
        _raw:     set,
    };
}

function makeEl(...classes: string[]) {
    return { classList: makeClassList(...classes) } as any;
}

function makeBtn(...classes: string[]) {
    const el = makeEl(...classes);
    el.disabled = false;
    return el;
}

function makeUI() {
    return {
        processingModeSwitch: { checked: false } as any,
        batchUiElements:      [makeEl(), makeEl()],
        toggleButtons:        [makeBtn(), makeBtn(), makeBtn(), makeBtn(), makeBtn()],

        drawingTools:   makeEl(),
        thermalTools:   makeEl(),
        fixturesTools:  makeEl(),

        drawingUiElements:  [makeEl(), makeEl()],
        thermalUiElements:  [makeEl(), makeEl()],
        fixturesUiElements: [makeEl(), makeEl()],

        penBtn:      makeBtn(),
        squareBtn:   makeBtn(),
        circleBtn:   makeBtn(),
        triangleBtn: makeBtn(),
        lineBtn:     makeBtn(),
        markerBtn:   makeBtn(),
        heatAreaBtn: makeBtn(),

        roundBrushBtn:   makeBtn(),
        squareBrushBtn:  makeBtn(),
        eraserBrushBtn:  makeBtn(),
    } as any;
}

function makeCM() {
    return {
        disableDrawing:        vi.fn(),
        enableDrawing:         vi.fn(),
        setShapeType:          vi.fn(),
        enableFixturesMode:    vi.fn(),
        disableFixturesMode:   vi.fn(),
        enableMarkerMode:      vi.fn(),
        disableMarkerMode:     vi.fn(),
        enableHeatAreaMode:    vi.fn(),
        disableHeatAreaMode:   vi.fn(),
        showMarkers:           vi.fn(),
        hideMarkers:           vi.fn(),
        showFixtures:          vi.fn(),
        hasFixtures:           vi.fn(() => false),
        canApplyFixtures:      vi.fn(() => false),
        clearFixtures:         vi.fn(),
    } as any;
}

// ---------------------------------------------------------------------------
describe('ModeManager – toggleMode (real-time ↔ batch)', () => {
    let ui: any, state: AppState, cm: any, mm: ModeManager;
    let updateDraw: Mock, updateFix: Mock, updateTherm: Mock;

    beforeEach(() => {
        ui    = makeUI();
        state = createAppState();
        cm    = makeCM();
        updateDraw  = vi.fn();
        updateFix   = vi.fn();
        updateTherm = vi.fn();
        mm    = new ModeManager(ui, state, () => cm, updateDraw, updateFix, updateTherm);
    });

    it('switching to real-time hides batch elements', () => {
        ui.processingModeSwitch.checked = true;
        mm.toggleMode();

        ui.batchUiElements.forEach((el: any) =>
            expect(el.classList.contains('hidden-mode')).toBe(true)
        );
    });

    it('switching to batch shows batch elements', () => {
        ui.processingModeSwitch.checked = false;
        mm.toggleMode();

        ui.batchUiElements.forEach((el: any) =>
            expect(el.classList.contains('hidden-mode')).toBe(false)
        );
    });

    it('toggleMode resets selectedShape and drawnShapeType', () => {
        state.selectedShape  = 'circle';
        state.drawnShapeType = 'circle';
        mm.toggleMode();
        expect(state.selectedShape).toBeNull();
        expect(state.drawnShapeType).toBeNull();
    });

    it('toggleMode clears "selected" from every toggle button', () => {
        ui.toggleButtons[2].classList.add('selected');
        mm.toggleMode();
        ui.toggleButtons.forEach((btn: any) =>
            expect(btn.classList.contains('selected')).toBe(false)
        );
    });

    it('toggleMode calls updateDrawButtonState', () => {
        mm.toggleMode();
        expect(updateDraw).toHaveBeenCalled();
    });
});

// ---------------------------------------------------------------------------
describe('ModeManager – switchMode → fixtures', () => {
    let ui: any, state: AppState, cm: any, mm: ModeManager;
    let updateFix: Mock;

    beforeEach(() => {
        ui    = makeUI();
        state = createAppState();
        state.currentMode = 'drawing';
        cm    = makeCM();
        updateFix = vi.fn();
        mm    = new ModeManager(ui, state, () => cm, vi.fn(), updateFix, vi.fn());
    });

    it('sets currentMode to fixtures', () => {
        mm.switchMode('fixturesBtn');
        expect(state.currentMode).toBe('fixtures');
    });

    it('shows fixturesTools, hides drawing and thermal tools', () => {
        mm.switchMode('fixturesBtn');

        expect(ui.fixturesTools.classList.contains('hidden')).toBe(false);
        expect(ui.drawingTools.classList.contains('hidden')).toBe(true);
        expect(ui.thermalTools.classList.contains('hidden')).toBe(true);
    });

    it('calls cm.enableFixturesMode and disableDrawing', () => {
        mm.switchMode('fixturesBtn');
        expect(cm.enableFixturesMode).toHaveBeenCalled();
        expect(cm.disableDrawing).toHaveBeenCalled();
    });

    it('calls updateFixturesButtonState', () => {
        mm.switchMode('fixturesBtn');
        expect(updateFix).toHaveBeenCalled();
    });
});

// ---------------------------------------------------------------------------
describe('ModeManager – switchMode → thermal', () => {
    let ui: any, state: AppState, cm: any, mm: ModeManager;
    let updateTherm: Mock;

    beforeEach(() => {
        ui    = makeUI();
        state = createAppState();
        state.currentMode = 'drawing';
        cm    = makeCM();
        updateTherm = vi.fn();
        mm    = new ModeManager(ui, state, () => cm, vi.fn(), vi.fn(), updateTherm);
    });

    it('sets currentMode to thermal', () => {
        mm.switchMode('thermalBtn');
        expect(state.currentMode).toBe('thermal');
    });

    it('shows thermalTools, hides drawing and fixtures tools', () => {
        mm.switchMode('thermalBtn');
        expect(ui.thermalTools.classList.contains('hidden')).toBe(false);
        expect(ui.drawingTools.classList.contains('hidden')).toBe(true);
        expect(ui.fixturesTools.classList.contains('hidden')).toBe(true);
    });

    it('calls cm.showMarkers', () => {
        mm.switchMode('thermalBtn');
        expect(cm.showMarkers).toHaveBeenCalled();
    });

    it('restores marker mode if selectedShape was marker', () => {
        state.selectedShape = 'marker';
        mm.switchMode('thermalBtn');
        expect(cm.enableMarkerMode).toHaveBeenCalled();
        expect(ui.markerBtn.classList.contains('selected')).toBe(true);
    });

    it('shows fixtures overlay (non-editable) if fixtures exist', () => {
        cm.hasFixtures.mockReturnValue(true);
        mm.switchMode('thermalBtn');
        expect(cm.showFixtures).toHaveBeenCalled();
    });

    it('calls updateThermalButtonState', () => {
        mm.switchMode('thermalBtn');
        expect(updateTherm).toHaveBeenCalled();
    });
});

// ---------------------------------------------------------------------------
describe('ModeManager – switchMode → drawing', () => {
    let ui: any, state: AppState, cm: any, mm: ModeManager;
    let updateDraw: Mock;

    beforeEach(() => {
        ui    = makeUI();
        state = createAppState();
        state.currentMode = 'thermal';   // coming from thermal
        cm    = makeCM();
        updateDraw = vi.fn();
        mm    = new ModeManager(ui, state, () => cm, updateDraw, vi.fn(), vi.fn());
    });

    it('sets currentMode to drawing', () => {
        mm.switchMode('drawingBtn');
        expect(state.currentMode).toBe('drawing');
    });

    it('shows drawingTools, hides thermal and fixtures tools', () => {
        mm.switchMode('drawingBtn');
        expect(ui.drawingTools.classList.contains('hidden')).toBe(false);
        expect(ui.thermalTools.classList.contains('hidden')).toBe(true);
        expect(ui.fixturesTools.classList.contains('hidden')).toBe(true);
    });

    it('disables marker and heat-area modes', () => {
        mm.switchMode('drawingBtn');
        expect(cm.disableMarkerMode).toHaveBeenCalled();
        expect(cm.disableHeatAreaMode).toHaveBeenCalled();
    });

    it('re-arms shape tool when selectedShape is set but nothing drawn', () => {
        state.selectedShape  = 'triangle';
        state.drawnShapeType = null;
        mm.switchMode('drawingBtn');

        expect(cm.setShapeType).toHaveBeenCalledWith('triangle');
        expect(cm.enableDrawing).toHaveBeenCalled();
        expect(ui.triangleBtn.classList.contains('selected')).toBe(true);
    });

    it('locks tools when a shape is already drawn', () => {
        state.drawnShapeType = 'line';
        mm.switchMode('drawingBtn');

        expect(cm.disableDrawing).toHaveBeenCalled();
        expect(ui.lineBtn.classList.contains('selected')).toBe(true);
    });

    it('shows fixtures overlay if they exist', () => {
        cm.hasFixtures.mockReturnValue(true);
        mm.switchMode('drawingBtn');
        expect(cm.showFixtures).toHaveBeenCalled();
    });

    it('calls updateDrawButtonState', () => {
        mm.switchMode('drawingBtn');
        expect(updateDraw).toHaveBeenCalled();
    });
});

// ---------------------------------------------------------------------------
describe('ModeManager – switchMode teardown paths', () => {
    let ui: any, state: AppState, cm: any, mm: ModeManager;

    beforeEach(() => {
        ui    = makeUI();
        state = createAppState();
        cm    = makeCM();
        mm    = new ModeManager(ui, state, () => cm, vi.fn(), vi.fn(), vi.fn());
    });

    it('leaving fixtures (with unapplied fixtures) clears them', () => {
        state.currentMode = 'fixtures';
        cm.canApplyFixtures.mockReturnValue(true);

        mm.switchMode('drawingBtn');   // leaving fixtures

        expect(cm.clearFixtures).toHaveBeenCalled();
        expect(state.selectedBrushType).toBeNull();
        expect(state.isEraserActive).toBe(false);
    });

    it('leaving fixtures when fixtures are already applied does NOT clear', () => {
        state.currentMode = 'fixtures';
        cm.canApplyFixtures.mockReturnValue(false);   // already applied

        mm.switchMode('thermalBtn');
        expect(cm.clearFixtures).not.toHaveBeenCalled();
    });

    it('leaving thermal disables marker mode and removes selected class', () => {
        state.currentMode = 'thermal';
        ui.markerBtn.classList.add('selected');

        mm.switchMode('drawingBtn');

        expect(cm.disableMarkerMode).toHaveBeenCalled();
        expect(ui.markerBtn.classList.contains('selected')).toBe(false);
    });

    it('staying in fixtures (fixturesBtn → fixturesBtn) does NOT run fixtures teardown', () => {
        state.currentMode = 'fixtures';
        cm.canApplyFixtures.mockReturnValue(true);

        mm.switchMode('fixturesBtn');   // same mode
        expect(cm.clearFixtures).not.toHaveBeenCalled();
    });
});

// ---------------------------------------------------------------------------
describe('ModeManager – highlightShapeBtn', () => {
    let ui: any, mm: ModeManager;

    beforeEach(() => {
        ui = makeUI();
        mm = new ModeManager(ui, createAppState(), () => null, vi.fn(), vi.fn(), vi.fn());
    });

    it('adds "selected" to the freehand (pen) button', () => {
        mm.highlightShapeBtn('freehand');
        expect(ui.penBtn.classList.contains('selected')).toBe(true);
    });

    it('adds "selected" to square button', () => {
        mm.highlightShapeBtn('square');
        expect(ui.squareBtn.classList.contains('selected')).toBe(true);
    });

    it('adds "selected" to circle button', () => {
        mm.highlightShapeBtn('circle');
        expect(ui.circleBtn.classList.contains('selected')).toBe(true);
    });

    it('adds "selected" to triangle button', () => {
        mm.highlightShapeBtn('triangle');
        expect(ui.triangleBtn.classList.contains('selected')).toBe(true);
    });

    it('adds "selected" to line button', () => {
        mm.highlightShapeBtn('line');
        expect(ui.lineBtn.classList.contains('selected')).toBe(true);
    });
});
