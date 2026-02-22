//frontend/src/features/settings/SettingsManager.ts

import { UIRegistry }        from '../../core/UIRegistry';
import { AppState }          from '../../core/AppState';
import { CanvasManager }     from '../../ui/CanvasManager';
import { HardwareController } from '../hardware/HardwareController';

/**
 * SettingsManager
 *
 * Owns everything related to the settings modal, layout-position
 * persistence (menu top/bottom, sidebar left/right), and the
 * responsive window-scaling logic.
 */
export class SettingsManager {
    private readonly TARGET_WIDTH = 1700;

    constructor(
        private readonly ui:               UIRegistry,
        private readonly state:            AppState,
        private getCanvasManager:          () => CanvasManager | null,
        private readonly hardware:         HardwareController,
        private updateDrawButtonState:     () => void,
    ) {}

    // ===================================================================
    // Settings modal
    // ===================================================================

    /**
     * Opens the settings popup.  As a safety precaution it clears any
     * in-progress drawing and turns off both hardware outputs before
     * the modal becomes visible.
     */
    openSettings(): void {
        const cm = this.getCanvasManager();

        // Clear any active drawing to avoid state conflicts while modal is open
        if (cm && cm.hasShape()) {
            cm.clearDrawing();
            this.state.selectedShape  = null;
            this.state.drawnShapeType = null;
            this.ui.toggleButtons.forEach(btn => btn.classList.remove('selected'));
            this.updateDrawButtonState();
        }

        // Disable primary action buttons while settings are visible
        [this.ui.clearBtn, this.ui.prepareBtn, this.ui.robotBtn, this.ui.laserBtn]
            .forEach(btn => btn.disabled = true);

        // Safety: guarantee hardware is off
        this.hardware.changeLaserState(false);
        this.hardware.changeRobotState(false);

        this.ui.settingsPopup.classList.add('active');
        this.ui.overlay.classList.add('active');
    }

    closeSettings(): void {
        this.ui.settingsPopup.classList.remove('active');
        this.ui.overlay.classList.remove('active');

        // Re-enable hardware buttons
        [this.ui.robotBtn, this.ui.laserBtn].forEach(btn => btn.disabled = false);
    }

    // ===================================================================
    // Layout-position controls
    // ===================================================================

    setMenuPosition(position: 'top' | 'bottom'): void {
        if (position === 'bottom') {
            document.body.classList.add('menu-bottom');
            this.ui.layoutBottomBtn.classList.add('active');
            this.ui.layoutTopBtn.classList.remove('active');
        } else {
            document.body.classList.remove('menu-bottom');
            this.ui.layoutTopBtn.classList.add('active');
            this.ui.layoutBottomBtn.classList.remove('active');
        }
        localStorage.setItem('menuPosition', position);
    }

    setSidebarPosition(position: 'left' | 'right'): void {
        if (position === 'right') {
            document.body.classList.add('sidebar-right');
            this.ui.layoutRightBtn.classList.add('active');
            this.ui.layoutLeftBtn.classList.remove('active');
        } else {
            document.body.classList.remove('sidebar-right');
            this.ui.layoutLeftBtn.classList.add('active');
            this.ui.layoutRightBtn.classList.remove('active');
        }
        localStorage.setItem('sidebarPosition', position);
    }

    /**
     * Reads persisted menu/sidebar positions from localStorage and
     * applies them.  Safe to call multiple times (idempotent).
     */
    restoreLayoutPositions(): void {
        const menuPos    = localStorage.getItem('menuPosition')    || 'top';
        const sidebarPos = localStorage.getItem('sidebarPosition') || 'left';

        if (menuPos === 'bottom') {
            document.body.classList.add('menu-bottom');
            this.ui.layoutBottomBtn.classList.add('active');
            this.ui.layoutTopBtn.classList.remove('active');
        }
        if (sidebarPos === 'right') {
            document.body.classList.add('sidebar-right');
            this.ui.layoutRightBtn.classList.add('active');
            this.ui.layoutLeftBtn.classList.remove('active');
        }
    }

    // ===================================================================
    // Responsive window scaling
    // ===================================================================

    /**
     * If the viewport is narrower than TARGET_WIDTH the entire app container
     * is uniformly scaled down so that nothing clips or overflows.
     * Called on init and on every 'resize' event.
     */
    handleResize(): void {
        const scaler = document.getElementById('app-scaler');
        if (!scaler) return;

        const windowWidth  = window.innerWidth;
        const windowHeight = window.innerHeight;

        if (windowWidth < this.TARGET_WIDTH) {
            const scale = windowWidth / this.TARGET_WIDTH;
            scaler.style.width     = `${this.TARGET_WIDTH}px`;
            scaler.style.height    = `${windowHeight / scale}px`;
            scaler.style.transform = `scale(${scale})`;
        } else {
            scaler.style.width     = '100%';
            scaler.style.height    = '100%';
            scaler.style.transform = 'none';
        }
    }
}