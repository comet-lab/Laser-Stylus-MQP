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
    latestRealTimePos: { x: number; y: number } | null;

    // --- Raster / Fill State ---
    fillEnabled: boolean;
    selectedRasterPattern: 'line_raster' | 'spiral_raster' | null;

    // --- Application Mode ---
    currentMode: 'drawing' | 'thermal' | 'fixtures';

    // --- Fixture Brush State ---
    selectedBrushType: 'round' | 'square' | null;
    isEraserActive: boolean;
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
        latestRealTimePos: null,

        fillEnabled:            false,
        selectedRasterPattern:  'line_raster',

        currentMode: 'drawing',

        selectedBrushType: null,
        isEraserActive:    false,
    };
}