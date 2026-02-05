// tests/HardwareController.test.ts
import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest';
import { HardwareController } from '../src/features/hardware/HardwareController';
import { createAppState, AppState } from '../src/core/AppState';

// ---------------------------------------------------------------------------
// Helpers – minimal UI stubs that mirror the fields HardwareController touches
// ---------------------------------------------------------------------------
function makeBtn(activeClasses: string[] = []): HTMLButtonElement {
    const btn = document.createElement('button') as any;
    btn.style = { pointerEvents: 'auto' };
    activeClasses.forEach(c => btn.classList.add(c));
    return btn;
}

function makeUI(laserActive = false, robotActive = false) {
    return {
        laserBtn: makeBtn(laserActive  ? ['active'] : []),
        robotBtn: makeBtn(robotActive  ? ['active'] : []),
    } as any;   // cast – we only use these two fields
}

function makeWS() {
    return { updateState: vi.fn() } as any;
}

// ---------------------------------------------------------------------------
describe('HardwareController – changeLaserState', () => {
    let ui: any, state: AppState, ws: any, ctrl: HardwareController;

    beforeEach(() => {
        vi.useFakeTimers();
        ui    = makeUI();
        state = createAppState();
        ws    = makeWS();
        ctrl  = new HardwareController(ui, state, ws);
    });
    afterEach(() => vi.useRealTimers());

    it('sends isLaserOn:true and locks pointer', () => {
        ctrl.changeLaserState(true);
        expect(ws.updateState).toHaveBeenCalledWith(expect.objectContaining({ isLaserOn: true }));
        expect(ui.laserBtn.style.pointerEvents).toBe('none');
    });

    it('sends isLaserOn:false', () => {
        ctrl.changeLaserState(false);
        expect(ws.updateState).toHaveBeenCalledWith(expect.objectContaining({ isLaserOn: false }));
    });

    it('safety interlock: turning laser OFF while robot is active also sends isRobotOn:false', () => {
        ui.robotBtn.classList.add('active');   // simulate robot ON in UI
        ctrl.changeLaserState(false);

        expect(ws.updateState).toHaveBeenCalledWith(
            expect.objectContaining({ isLaserOn: false, isRobotOn: false })
        );
        // Robot button pointer also locked
        expect(ui.robotBtn.style.pointerEvents).toBe('none');
    });

    it('NO interlock when robot is NOT active and laser turns off', () => {
        ui.robotBtn.classList.remove('active'); 
        ctrl.changeLaserState(false);
        // robotBtn has no 'active' class
        ctrl.changeLaserState(false);
        expect(ws.updateState).toHaveBeenCalledWith({ isLaserOn: false });
        // robotBtn pointer untouched
        expect(ui.robotBtn.style.pointerEvents).not.toBe('none');
    });

    it('safeguard timeout re-enables laser pointer after 2 s', () => {
        ctrl.changeLaserState(true);
        expect(ui.laserBtn.style.pointerEvents).toBe('none');
        vi.advanceTimersByTime(2000);
        expect(ui.laserBtn.style.pointerEvents).toBe('auto');
    });

    it('repeated calls clear the previous timeout before setting a new one', () => {
        ctrl.changeLaserState(true);
        const firstTimeout = state.laserConfirmationTimeout;
        ctrl.changeLaserState(false);
        // The first timeout should have been cleared (new one set)
        expect(state.laserConfirmationTimeout).not.toBe(firstTimeout);
    });
});

// ---------------------------------------------------------------------------
describe('HardwareController – changeRobotState', () => {
    let ui: any, state: AppState, ws: any, ctrl: HardwareController;

    beforeEach(() => {
        vi.useFakeTimers();
        ui    = makeUI();
        state = createAppState();
        ws    = makeWS();
        ctrl  = new HardwareController(ui, state, ws);
    });
    afterEach(() => vi.useRealTimers());

    it('sends isRobotOn:true and locks pointer', () => {
        ctrl.changeRobotState(true);
        expect(ws.updateState).toHaveBeenCalledWith(expect.objectContaining({ isRobotOn: true }));
        expect(ui.robotBtn.style.pointerEvents).toBe('none');
    });

    it('safety interlock: turning robot OFF while laser is active also sends isLaserOn:false', () => {
        ui.laserBtn.classList.add('active');
        ctrl.changeRobotState(false);

        expect(ws.updateState).toHaveBeenCalledWith(
            expect.objectContaining({ isRobotOn: false, isLaserOn: false })
        );
        expect(ui.laserBtn.style.pointerEvents).toBe('none');
    });

    it('safeguard timeout re-enables robot pointer after 2 s', () => {
        ctrl.changeRobotState(true);
        vi.advanceTimersByTime(2000);
        expect(ui.robotBtn.style.pointerEvents).toBe('auto');
    });
});

// ---------------------------------------------------------------------------
describe('HardwareController – applyServerLaserState / applyServerRobotState', () => {
    let ui: any, state: AppState, ws: any, ctrl: HardwareController;

    beforeEach(() => {
        vi.useFakeTimers();
        ui    = makeUI();
        state = createAppState();
        ws    = makeWS();
        ctrl  = new HardwareController(ui, state, ws);
    });
    afterEach(() => vi.useRealTimers());

    it('applyServerLaserState(true) adds "active" class', () => {
        ctrl.applyServerLaserState(true);
        expect(ui.laserBtn.classList.contains('active')).toBe(true);
    });

    it('applyServerLaserState(false) removes "active" class', () => {
        ui.laserBtn.classList.add('active');
        ctrl.applyServerLaserState(false);
        expect(ui.laserBtn.classList.contains('active')).toBe(false);
    });

    it('applyServerLaserState clears the pending confirmation timeout and re-enables pointer', () => {
        // Simulate an in-flight toggle
        ctrl.changeLaserState(true);   // sets timeout, locks pointer
        expect(ui.laserBtn.style.pointerEvents).toBe('none');

        // Server ACK arrives
        ctrl.applyServerLaserState(true);
        expect(ui.laserBtn.style.pointerEvents).toBe('auto');
        expect(state.laserConfirmationTimeout).toBeNull();
    });

    it('applyServerRobotState mirrors the same behaviour for robot', () => {
        ctrl.changeRobotState(true);
        ctrl.applyServerRobotState(true);
        expect(ui.robotBtn.style.pointerEvents).toBe('auto');
        expect(state.robotConfirmationTimeout).toBeNull();
        expect(ui.robotBtn.classList.contains('active')).toBe(true);
    });

    it('applyServer* is safe to call when no timeout is pending (no-op)', () => {
        expect(() => ctrl.applyServerLaserState(false)).not.toThrow();
        expect(() => ctrl.applyServerRobotState(false)).not.toThrow();
    });
});
