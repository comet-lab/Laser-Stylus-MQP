//frontend/src/core/AppState.ts

import { ShapeType } from '../ui/types';

/**
 * AppState
 *
 * Single source of truth for all mutable runtime state owned by the controller.
 * Every sub-module that needs to read or mutate state receives a reference to
 * this same object — no copies, no duplicated flags.
 */

export interface AppState {
    // --- Hardware Confirmation Timeouts ---
    // Prevents rapid re-toggling while awaiting server ACK.
    laserConfirmationTimeout: number | null;
    robotConfirmationTimeout: number | null;

    // --- Drawing State ---
    selectedShape: ShapeType | 'marker' | null;  // Tool currently selected by the user
    drawnShapeType: ShapeType | null;            // Shape that is currently rendered on canvas

    // --- Real-time Mode State ---
    isRealTimeDrawing: boolean;
    activeRealTimeTool: 'pen' | null;
    latestRealTimePos: { x: number; y: number } | null;

    // --- Raster / Fill State ---
    fillEnabled: boolean;
    //TODO: Remove entirely, unless we want to add more raster patterns in the future.
    selectedRasterPattern: 'line_raster' | null;

    // --- Application Mode ---
    currentMode: 'drawing' | 'thermal' | 'fixtures';

    // --- Fixture Brush State ---
    selectedBrushType: 'round' | 'square' | null;
    isEraserActive: boolean;

    hasBackupShape: boolean; //Flag to indicate if a backup shape exists for restoration
    backupShapeType: ShapeType | null;
}

/**
 * Creates and returns a fresh AppState with all defaults.
 */
export function createAppState(): AppState {
    return {
        laserConfirmationTimeout: null,
        robotConfirmationTimeout: null,

        selectedShape:    null,
        drawnShapeType:   null,

        isRealTimeDrawing: false,
        activeRealTimeTool: null,
        latestRealTimePos: null,

        fillEnabled:            false,
        selectedRasterPattern:  'line_raster',

        currentMode: 'drawing',

        selectedBrushType: null,
        isEraserActive:    false,

        hasBackupShape: false,
        backupShapeType: null
    };
}