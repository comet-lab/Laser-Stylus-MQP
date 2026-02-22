// vitest.config.ts
import { defineConfig } from 'vitest/config';

export default defineConfig({
    test: {
        // happy-dom is fully ESM-native — avoids the parse5/jsdom CJS conflict
        environment: 'happy-dom',
        include: ['tests/**/*.test.ts'],
        globals: true,
        pool: 'threads',
    },
});