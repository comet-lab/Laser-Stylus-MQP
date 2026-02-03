// tests/WebSocketHandler.test.ts
import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest';
import { WebSocketHandler, WebSocketMessage } from '../src/services/WebSocketHandler';

// ---------------------------------------------------------------------------
// Minimal WebSocket stub that captures handlers and lets us fire them manually
// ---------------------------------------------------------------------------
class MockWebSocket {
    static OPEN  = 1;
    static CLOSED = 3;

    readyState: number;
    url: string;

    onopen:    ((ev: any)    => void) | null = null;
    onmessage: ((ev: any)    => void) | null = null;
    onclose:   ((ev: any)    => void) | null = null;
    onerror:   ((ev: any)    => void) | null = null;

    private sentMessages: string[] = [];

    constructor(url: string) {
        this.url = url;
        // Start in OPEN so tests can send immediately; individual tests may override
        this.readyState = MockWebSocket.OPEN;
    }

    send(data: string) { this.sentMessages.push(data); }
    close()            { this.readyState = MockWebSocket.CLOSED; }
    getSent()          { return this.sentMessages; }
}

let mockWsInstance: MockWebSocket | null = null;

beforeEach(() => {
    // Patch the global WebSocket constructor with our mock.
    // import.meta.env is provided by Vitest automatically; the URL value
    // is irrelevant because the mock never opens a real connection.
    (globalThis as any).WebSocket = class extends MockWebSocket {
        constructor(url: string) {
            super(url);
            mockWsInstance = this as any;
        }
    };
});

afterEach(() => {
    delete (globalThis as any).WebSocket;
    mockWsInstance = null;
    vi.restoreAllMocks();
});

// ---------------------------------------------------------------------------
describe('WebSocketHandler – connection lifecycle', () => {
    it('connect() creates a WebSocket and stores it', () => {
        const handler = new WebSocketHandler(null);
        handler.connect();
        expect(mockWsInstance).toBeTruthy();
    });

    it('connect() closes an existing connection before opening a new one', () => {
        const handler = new WebSocketHandler(null);
        handler.connect();
        const first = mockWsInstance;
        // Spy on the close of the first WS
        const closeSpy = vi.spyOn(first!, 'close');
        handler.connect();                        // second connect
        expect(closeSpy).toHaveBeenCalled();
        expect(mockWsInstance).not.toBe(first);   // new instance
    });

    it('disconnect() closes the socket and nulls the reference', () => {
        const handler = new WebSocketHandler(null);
        handler.connect();
        const ws = mockWsInstance!;
        handler.disconnect();
        expect(ws.readyState).toBe(MockWebSocket.CLOSED);
        // Calling disconnect again should not throw
        expect(() => handler.disconnect()).not.toThrow();
    });
});

// ---------------------------------------------------------------------------
describe('WebSocketHandler – message parsing', () => {
    it('calls onStateUpdate with parsed JSON when a valid message arrives', () => {
        const handler = new WebSocketHandler(null);
        const spy = vi.fn();
        handler.onStateUpdate = spy;
        handler.connect();

        const payload: WebSocketMessage = {
            x: 10, y: 20, z: 0, rx: 0, ry: 0, rz: 0,
            laserX: 100, laserY: 200,
            averageHeat: 45.6, beamWaist: 0.5, speed: 10,
            isLaserOn: true, isRobotOn: false,
            isTransformedViewOn: false, pathEvent: null,
        };

        // Simulate server push
        mockWsInstance!.onmessage!({ data: JSON.stringify(payload) });

        expect(spy).toHaveBeenCalledTimes(1);
        expect(spy).toHaveBeenCalledWith(payload);
    });

    it('does NOT throw when JSON is malformed – logs instead', () => {
        const logEl = document.createElement('div');
        const handler = new WebSocketHandler(logEl);
        handler.connect();

        expect(() => {
            mockWsInstance!.onmessage!({ data: 'not-json{{{}' });
        }).not.toThrow();

        // The log element should contain an error string
        expect(logEl.textContent).toMatch(/Error parsing/i);
    });

    it('does not call onStateUpdate when callback is null', () => {
        const handler = new WebSocketHandler(null);
        handler.onStateUpdate = null;
        handler.connect();
        // Should not throw even though callback is null
        expect(() => {
            mockWsInstance!.onmessage!({ data: '{"x":1}' });
        }).not.toThrow();
    });
});

// ---------------------------------------------------------------------------
describe('WebSocketHandler – send / updateState guards', () => {
    it('send() returns true and transmits when OPEN', () => {
        const handler = new WebSocketHandler(null);
        handler.connect();
        mockWsInstance!.readyState = MockWebSocket.OPEN;

        expect(handler.send('hello')).toBe(true);
        expect(mockWsInstance!.getSent()).toContain('hello');
    });

    it('send() returns false and does not transmit when NOT OPEN', () => {
        const handler = new WebSocketHandler(null);
        handler.connect();
        mockWsInstance!.readyState = 0; // CONNECTING

        expect(handler.send('hello')).toBe(false);
        expect(mockWsInstance!.getSent()).toHaveLength(0);
    });

    it('updateState() serialises a partial payload and sends it', () => {
        const handler = new WebSocketHandler(null);
        handler.connect();
        mockWsInstance!.readyState = MockWebSocket.OPEN;

        const partial = { isLaserOn: true, speed: 42 };
        expect(handler.updateState(partial)).toBe(true);
        expect(JSON.parse(mockWsInstance!.getSent()[0])).toEqual(partial);
    });

    it('updateState() returns false when socket is not open', () => {
        const handler = new WebSocketHandler(null);
        handler.connect();
        mockWsInstance!.readyState = 0;

        expect(handler.updateState({ x: 5 })).toBe(false);
    });

    it('send() before connect() returns false (ws is null)', () => {
        const handler = new WebSocketHandler(null);
        expect(handler.send('early')).toBe(false);
    });

    it('updateState() before connect() returns false', () => {
        const handler = new WebSocketHandler(null);
        expect(handler.updateState({ x: 1 })).toBe(false);
    });
});

// ---------------------------------------------------------------------------
describe('WebSocketHandler – logging to output element', () => {
    it('appends messages to the provided element', () => {
        const el = document.createElement('pre');
        const handler = new WebSocketHandler(el);
        handler.connect();

        // onclose fires a log
        mockWsInstance!.onclose!({});
        expect(el.textContent).toMatch(/Disconnected/i);
    });

    it('does not throw when output element is null (console path)', () => {
        const handler = new WebSocketHandler(null);
        handler.connect();
        expect(() => mockWsInstance!.onclose!({})).not.toThrow();
    });
});