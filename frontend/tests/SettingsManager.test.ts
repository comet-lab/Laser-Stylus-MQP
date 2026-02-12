// tests/SettingsManager.test.ts
import { describe, it, expect, beforeEach, afterEach, vi, type Mock } from 'vitest';
import { SettingsManager } from '../src/features/settings/SettingsManager';
import { createAppState, AppState } from '../src/core/AppState';

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
function makeClassList(...initial: string[]) {
    const set = new Set(initial);
    return {
        add:      (c: string) => set.add(c),
        remove:   (c: string) => set.delete(c),
        contains: (c: string) => set.has(c),
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
        settingsPopup:  makeEl(),
        overlay:        makeEl(),

        clearBtn:    makeBtn(),
        prepareBtn:  makeBtn(),
        robotBtn:    makeBtn(),
        laserBtn:    makeBtn(),

        toggleButtons: [makeBtn(), makeBtn()],

        layoutTopBtn:    makeBtn(),
        layoutBottomBtn: makeBtn(),
        layoutLeftBtn:   makeBtn(),
        layoutRightBtn:  makeBtn(),
    } as any;
}

function makeCM(hasShape = false) {
    return {
        hasShape:     vi.fn(() => hasShape),
        clearDrawing: vi.fn(),
    } as any;
}

function makeHW() {
    return {
        changeLaserState: vi.fn(),
        changeRobotState: vi.fn(),
    } as any;
}

// ---------------------------------------------------------------------------
describe('SettingsManager – openSettings', () => {
    let ui: any, state: AppState, cm: any, hw: any, updateDraw: Mock, sm: SettingsManager;

    beforeEach(() => {
        ui         = makeUI();
        state      = createAppState();
        cm         = makeCM(false);
        hw         = makeHW();
        updateDraw = vi.fn();
        sm         = new SettingsManager(ui, state, () => cm, hw, updateDraw);
    });

    it('shows the popup and overlay', () => {
        sm.openSettings();
        expect(ui.settingsPopup.classList.contains('active')).toBe(true);
        expect(ui.overlay.classList.contains('active')).toBe(true);
    });

    it('disables clear / prepare / robot / laser buttons', () => {
        sm.openSettings();
        expect(ui.clearBtn.disabled).toBe(true);
        expect(ui.prepareBtn.disabled).toBe(true);
        expect(ui.robotBtn.disabled).toBe(true);
        expect(ui.laserBtn.disabled).toBe(true);
    });

    it('sends laser=false and robot=false as a safety precaution', () => {
        sm.openSettings();
        expect(hw.changeLaserState).toHaveBeenCalledWith(false);
        expect(hw.changeRobotState).toHaveBeenCalledWith(false);
    });

    it('if a shape exists it is cleared and tool state reset', () => {
        cm = makeCM(true);   // hasShape returns true
        sm = new SettingsManager(ui, state, () => cm, hw, updateDraw);
        state.selectedShape  = 'circle';
        state.drawnShapeType = 'circle';

        sm.openSettings();

        expect(cm.clearDrawing).toHaveBeenCalled();
        expect(state.selectedShape).toBeNull();
        expect(state.drawnShapeType).toBeNull();
        expect(updateDraw).toHaveBeenCalled();
    });

    it('does NOT call clearDrawing when no shape is on canvas', () => {
        sm.openSettings();
        expect(cm.clearDrawing).not.toHaveBeenCalled();
    });
});

// ---------------------------------------------------------------------------
describe('SettingsManager – closeSettings', () => {
    let ui: any, sm: SettingsManager;

    beforeEach(() => {
        ui = makeUI();
        sm = new SettingsManager(ui, createAppState(), () => makeCM(), makeHW(), vi.fn());
    });

    it('hides popup and overlay', () => {
        // First open it
        sm.openSettings();
        sm.closeSettings();
        expect(ui.settingsPopup.classList.contains('active')).toBe(false);
        expect(ui.overlay.classList.contains('active')).toBe(false);
    });

    it('re-enables robot and laser buttons', () => {
        sm.openSettings();   // disables them
        sm.closeSettings();
        expect(ui.robotBtn.disabled).toBe(false);
        expect(ui.laserBtn.disabled).toBe(false);
    });
});

// ---------------------------------------------------------------------------
describe('SettingsManager – layout position (menu top/bottom)', () => {
    let ui: any, sm: SettingsManager;

    beforeEach(() => {
        ui = makeUI();
        sm = new SettingsManager(ui, createAppState(), () => null, makeHW(), vi.fn());
        vi.spyOn(localStorage, 'setItem');
        vi.spyOn(localStorage, 'getItem').mockReturnValue(null);
    });
    afterEach(() => {
        vi.restoreAllMocks();
        document.body.classList.remove('menu-bottom');
    });

    it('setMenuPosition("bottom") adds menu-bottom to body, marks bottom active', () => {
        sm.setMenuPosition('bottom');
        expect(document.body.classList.contains('menu-bottom')).toBe(true);
        expect(ui.layoutBottomBtn.classList.contains('active')).toBe(true);
        expect(ui.layoutTopBtn.classList.contains('active')).toBe(false);
        expect(localStorage.setItem).toHaveBeenCalledWith('menuPosition', 'bottom');
        // cleanup
        document.body.classList.remove('menu-bottom');
    });

    it('setMenuPosition("top") removes menu-bottom from body', () => {
        document.body.classList.add('menu-bottom');
        sm.setMenuPosition('top');
        expect(document.body.classList.contains('menu-bottom')).toBe(false);
        expect(ui.layoutTopBtn.classList.contains('active')).toBe(true);
        expect(localStorage.setItem).toHaveBeenCalledWith('menuPosition', 'top');
    });
});

// ---------------------------------------------------------------------------
describe('SettingsManager – layout position (sidebar left/right)', () => {
    let ui: any, sm: SettingsManager;

    beforeEach(() => {
        ui = makeUI();
        sm = new SettingsManager(ui, createAppState(), () => null, makeHW(), vi.fn());
        vi.spyOn(localStorage, 'setItem');
    });
    afterEach(() => {
        vi.restoreAllMocks();
        document.body.classList.remove('sidebar-right');
    });

    it('setSidebarPosition("right") adds sidebar-right to body', () => {
        sm.setSidebarPosition('right');
        expect(document.body.classList.contains('sidebar-right')).toBe(true);
        expect(ui.layoutRightBtn.classList.contains('active')).toBe(true);
        expect(ui.layoutLeftBtn.classList.contains('active')).toBe(false);
        expect(localStorage.setItem).toHaveBeenCalledWith('sidebarPosition', 'right');
    });

    it('setSidebarPosition("left") removes sidebar-right', () => {
        document.body.classList.add('sidebar-right');
        sm.setSidebarPosition('left');
        expect(document.body.classList.contains('sidebar-right')).toBe(false);
        expect(ui.layoutLeftBtn.classList.contains('active')).toBe(true);
    });
});

// ---------------------------------------------------------------------------
describe('SettingsManager – restoreLayoutPositions', () => {
    let ui: any, sm: SettingsManager;

    afterEach(() => {
        vi.restoreAllMocks();
        document.body.classList.remove('menu-bottom');
        document.body.classList.remove('sidebar-right');
    });

    it('applies persisted bottom + right', () => {
        vi.spyOn(localStorage, 'getItem').mockImplementation((key) => {
            if (key === 'menuPosition')    return 'bottom';
            if (key === 'sidebarPosition') return 'right';
            return null;
        });
        ui = makeUI();
        sm = new SettingsManager(ui, createAppState(), () => null, makeHW(), vi.fn());

        sm.restoreLayoutPositions();

        expect(document.body.classList.contains('menu-bottom')).toBe(true);
        expect(document.body.classList.contains('sidebar-right')).toBe(true);
        expect(ui.layoutBottomBtn.classList.contains('active')).toBe(true);
        expect(ui.layoutRightBtn.classList.contains('active')).toBe(true);
    });

    it('applies defaults (top + left) when nothing is persisted', () => {
        vi.spyOn(localStorage, 'getItem').mockReturnValue(null);
        ui = makeUI();
        sm = new SettingsManager(ui, createAppState(), () => null, makeHW(), vi.fn());

        sm.restoreLayoutPositions();

        expect(document.body.classList.contains('menu-bottom')).toBe(false);
        expect(document.body.classList.contains('sidebar-right')).toBe(false);
    });

    it('is idempotent – calling twice produces same result', () => {
        vi.spyOn(localStorage, 'getItem').mockImplementation((key) =>
            key === 'menuPosition' ? 'bottom' : null
        );
        ui = makeUI();
        sm = new SettingsManager(ui, createAppState(), () => null, makeHW(), vi.fn());

        sm.restoreLayoutPositions();
        sm.restoreLayoutPositions();
        expect(document.body.classList.contains('menu-bottom')).toBe(true);
        document.body.classList.remove('menu-bottom');
    });
});

// ---------------------------------------------------------------------------
describe('SettingsManager – handleResize (responsive scaler)', () => {
    let sm: SettingsManager;
    let scaler: HTMLElement;

    beforeEach(() => {
        // Inject an #app-scaler element into the test DOM
        scaler = document.createElement('div');
        scaler.id = 'app-scaler';
        document.body.appendChild(scaler);

        sm = new SettingsManager(makeUI(), createAppState(), () => null, makeHW(), vi.fn());
    });

    afterEach(() => {
        scaler.remove();
        vi.unstubAllGlobals();
    });

    it('applies a scale transform when window is narrower than 1700 px', () => {
        // Simulate narrow window via vi.stubGlobal (happy-dom protects innerWidth as a getter)
        vi.stubGlobal('innerWidth',  850);
        vi.stubGlobal('innerHeight', 600);

        sm.handleResize();

        const scale = 850 / 1700;   // 0.5
        expect(scaler.style.transform).toBe(`scale(${scale})`);
        expect(scaler.style.width).toBe('1700px');
    });

    it('removes scaling when window is wide enough', () => {
        vi.stubGlobal('innerWidth',  1800);
        vi.stubGlobal('innerHeight', 900);

        sm.handleResize();

        expect(scaler.style.transform).toBe('none');
        expect(scaler.style.width).toBe('100%');
        expect(scaler.style.height).toBe('100%');
    });

    it('does not throw when #app-scaler is missing', () => {
        scaler.remove();
        expect(() => sm.handleResize()).not.toThrow();
    });
});