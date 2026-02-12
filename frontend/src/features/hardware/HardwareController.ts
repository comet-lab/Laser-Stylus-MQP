//frontend/src/features/hardware/HardwareController.ts

import { UIRegistry }              from '../../core/UIRegistry';
import { AppState }                from '../../core/AppState';
import { WebSocketHandler }        from '../../services/WebSocketHandler';

/**
 * HardwareController
 *
 * Owns all logic for toggling the laser and robot on/off, including the
 * safety interlock that turns the other device off when one is switched off
 * while both are active.  Manages the temporary pointer-event locks that
 * prevent double-clicks while the server ACK is in flight.
 */
export class HardwareController {
    constructor(
        private readonly ui:    UIRegistry,
        private readonly state: AppState,
        private readonly ws:    WebSocketHandler,
    ) {}

    // ---------------------------------------------------------------
    // Public – called by the main controller when the buttons fire
    // ---------------------------------------------------------------

    changeLaserState(newState: boolean): void {
        this.ui.laserBtn.style.pointerEvents = 'none';   // Lock UI

        const updates: any = { isLaserOn: newState };

        // Safety: turning laser OFF while robot is ON → also turn robot OFF
        if (!newState && this.ui.robotBtn.classList.contains('active')) {
            updates.isRobotOn = false;
            this.ui.robotBtn.style.pointerEvents = 'none';
            if (this.state.robotConfirmationTimeout) clearTimeout(this.state.robotConfirmationTimeout);
            this.state.robotConfirmationTimeout = setTimeout(
                () => { this.ui.robotBtn.style.pointerEvents = 'auto'; }, 2000
            );
        }

        this.ws.updateState(updates);

        // Safeguard: re-enable pointer if server never responds
        if (this.state.laserConfirmationTimeout) clearTimeout(this.state.laserConfirmationTimeout);
        this.state.laserConfirmationTimeout = setTimeout(
            () => { this.ui.laserBtn.style.pointerEvents = 'auto'; }, 2000
        );
    }

    changeRobotState(newState: boolean): void {
        this.ui.robotBtn.style.pointerEvents = 'none';   // Lock UI

        const updates: any = { isRobotOn: newState };

        // Safety: turning robot OFF while laser is ON → also turn laser OFF
        if (!newState && this.ui.laserBtn.classList.contains('active')) {
            updates.isLaserOn = false;
            this.ui.laserBtn.style.pointerEvents = 'none';
            if (this.state.laserConfirmationTimeout) clearTimeout(this.state.laserConfirmationTimeout);
            this.state.laserConfirmationTimeout = setTimeout(
                () => { this.ui.laserBtn.style.pointerEvents = 'auto'; }, 2000
            );
        }

        this.ws.updateState(updates);

        // Safeguard timeout
        if (this.state.robotConfirmationTimeout) clearTimeout(this.state.robotConfirmationTimeout);
        this.state.robotConfirmationTimeout = setTimeout(
            () => { this.ui.robotBtn.style.pointerEvents = 'auto'; }, 2000
        );
    }

    // ---------------------------------------------------------------
    // Called by syncUiToState when the server pushes a new state
    // ---------------------------------------------------------------

    /** Reconcile the laser button + timeout with the authoritative server value. */
    applyServerLaserState(isOn: boolean): void {
        this.ui.laserBtn.classList.toggle('active', isOn);
        if (this.state.laserConfirmationTimeout) {
            clearTimeout(this.state.laserConfirmationTimeout);
            this.state.laserConfirmationTimeout = null;
            this.ui.laserBtn.style.pointerEvents = 'auto';
        }
    }

    /** Reconcile the robot button + timeout with the authoritative server value. */
    applyServerRobotState(isOn: boolean): void {
        this.ui.robotBtn.classList.toggle('active', isOn);
        if (this.state.robotConfirmationTimeout) {
            clearTimeout(this.state.robotConfirmationTimeout);
            this.state.robotConfirmationTimeout = null;
            this.ui.robotBtn.style.pointerEvents = 'auto';
        }
    }
}