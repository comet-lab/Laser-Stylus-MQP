// frontend/src/ui/ToastManager.ts

export type ToastType = 'info' | 'warning' | 'error';

export interface ToastOptions {
    type?: ToastType;
    durationMs?: number;
    requireAck?: boolean;
    ackText?: string;
    onAcknowledge?: () => void;
}

export class ToastManager {
    private static container: HTMLElement | null = null;
    private static activeToasts: HTMLElement[] = [];

    private static initContainer() {
        if (this.container) return;

        this.container = document.createElement('div');
        this.container.id = 'toast-container';

        // --- Position & Layout ---
        this.container.style.position = 'absolute';
        this.container.style.top = '80px';
        this.container.style.left = '50%';
        this.container.style.transform = 'translateX(-50%)';
        this.container.style.zIndex = '9999';
        
        this.container.style.display = 'flex';
        this.container.style.flexDirection = 'column';
        this.container.style.gap = '12px';
        this.container.style.pointerEvents = 'none'; 
        this.container.style.width = 'auto';
        this.container.style.minWidth = '350px';
        this.container.style.maxWidth = '90%';

        const appScaler = document.getElementById('app-scaler');
        if (appScaler) {
            appScaler.appendChild(this.container);
        } else {
            document.body.appendChild(this.container);
        }
    }

    public static clearAll() {
        this.activeToasts.forEach(toast => this.dismiss(toast));
        this.activeToasts = [];
    }

    public static show(message: string, options: ToastOptions = {}) {
        this.initContainer();
        if (!this.container) return;

        const type = options.type || 'info';
        const requireAck = options.requireAck || false;
        const durationMs = options.durationMs || 4000;
        const ackText = options.ackText || 'OK';

        const toast = document.createElement('div');
        this.activeToasts.push(toast);
        
        // --- Core Tech Styling ---
        toast.style.fontFamily = "'IBM Plex Sans', sans-serif";
        toast.style.fontSize = '16px';
        toast.style.fontWeight = '500';
        toast.style.letterSpacing = '0.5px';
        toast.style.color = '#e0f2e0';
        toast.style.background = '#151b21';
        toast.style.border = '2px solid #2c343b';
        toast.style.boxShadow = '0 8px 16px rgba(0,0,0,0.6)';
        toast.style.padding = '16px 20px';
        toast.style.borderRadius = '6px';
        toast.style.pointerEvents = 'auto'; 
        toast.style.display = 'flex';
        toast.style.alignItems = 'center';
        toast.style.justifyContent = 'space-between';
        
        // --- Content ---
        const textSpan = document.createElement('span');
        textSpan.style.flexGrow = '1';
        textSpan.style.marginRight = '20px';
        textSpan.style.lineHeight = '1.4';
        textSpan.textContent = message; // Safer than innerHTML
        toast.appendChild(textSpan);

        // --- Status Strip ---
        let accentColor = '#70ff57';
        if (type === 'warning') accentColor = '#fa8500';
        if (type === 'error') accentColor = '#ef4444';
        toast.style.borderLeft = `6px solid ${accentColor}`;

        // --- Acknowledgment Button ---
        if (requireAck) {
            const btn = document.createElement('button');
            btn.textContent = ackText;
            btn.style.backgroundColor = accentColor;
            btn.style.color = type === 'warning' || type === 'info' ? '#000' : '#fff';
            btn.style.border = 'none';
            btn.style.padding = '8px 16px';
            btn.style.borderRadius = '4px';
            btn.style.fontWeight = '700';
            btn.style.cursor = 'pointer';
            btn.style.flexShrink = '0';
            btn.style.textTransform = 'uppercase';
            
            btn.onclick = () => {
                this.dismiss(toast);
                if (options.onAcknowledge) options.onAcknowledge();
            };
            toast.appendChild(btn);
        }

        // --- Animation Start State ---
        toast.style.opacity = '0';
        toast.style.transform = 'translateY(-10px) scale(0.95)';
        toast.style.transition = 'all 0.25s cubic-bezier(0.2, 0.8, 0.2, 1)';

        this.container.appendChild(toast);

        // Animate In
        requestAnimationFrame(() => {
            toast.style.opacity = '1';
            toast.style.transform = 'translateY(0) scale(1)';
        });

        // Auto-dismiss if not requiring interaction
        if (!requireAck) {
            setTimeout(() => {
                this.dismiss(toast);
            }, durationMs);
        }
    }

    private static dismiss(toast: HTMLElement) {
        toast.style.opacity = '0';
        toast.style.transform = 'translateY(-10px) scale(0.95)';
        toast.addEventListener('transitionend', () => {
            if (toast.parentNode) {
                toast.remove();
            }
            this.activeToasts = this.activeToasts.filter(t => t !== toast);
        });
    }
}