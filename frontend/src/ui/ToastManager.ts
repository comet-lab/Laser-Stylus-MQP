export type ToastType = 'info' | 'warning' | 'error';

export class ToastManager {
    private static container: HTMLElement | null = null;

    private static initContainer() {
        if (this.container) return;

        this.container = document.createElement('div');
        this.container.id = 'toast-container';

        // --- Position & Layout ---
        this.container.style.position = 'absolute';
        this.container.style.top = '70px'; // 60px header + 10px padding
        this.container.style.left = '50%';
        this.container.style.transform = 'translateX(-50%)';
        this.container.style.zIndex = '9999';
        
        this.container.style.display = 'flex';
        this.container.style.flexDirection = 'column';
        this.container.style.gap = '10px';
        this.container.style.pointerEvents = 'none'; // Clicks pass through the container gap
        this.container.style.width = 'auto';
        this.container.style.minWidth = '300px'; 
        this.container.style.maxWidth = '90%';

        // Attach to the scaler to match app zoom
        const appScaler = document.getElementById('app-scaler');
        if (appScaler) {
            appScaler.appendChild(this.container);
        } else {
            document.body.appendChild(this.container);
        }
    }

    public static show(message: string, type: ToastType = 'info', durationMs: number = 4000) {
        this.initContainer();
        if (!this.container) return;

        const toast = document.createElement('div');
        
        // --- Content ---
        // We can use a flex layout to allow for an icon in the future
        toast.innerHTML = `<span style="flex-grow:1">${message}</span>`;

        // --- Core Tech Styling ---
        toast.style.fontFamily = "'IBM Plex Sans', sans-serif";
        toast.style.fontSize = '14px';
        toast.style.fontWeight = '500';
        toast.style.letterSpacing = '0.5px';
        toast.style.color = '#e0f2e0'; // Your app's off-white text color
        
        toast.style.background = '#151b21'; // Dark panel background
        toast.style.border = '1px solid #2c343b'; // Thin tech border
        toast.style.boxShadow = '0 6px 12px rgba(0,0,0,0.5)';
        
        toast.style.padding = '12px 16px';
        toast.style.borderRadius = '4px'; // Sharper corners for industrial look
        toast.style.pointerEvents = 'auto'; 
        toast.style.display = 'flex';
        toast.style.alignItems = 'center';
        toast.style.justifyContent = 'space-between';
        
        // --- Status Strip (Left Border) ---
        // Instead of a full colored background, we use a neon strip on the left
        let accentColor = '#70ff57'; // Default Green
        if (type === 'warning') accentColor = '#fa8500'; // Orange
        if (type === 'error') accentColor = '#ef4444';   // Red

        toast.style.borderLeft = `4px solid ${accentColor}`;

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

        // Animate Out & Cleanup
        setTimeout(() => {
            toast.style.opacity = '0';
            toast.style.transform = 'translateY(-10px) scale(0.95)';
            toast.addEventListener('transitionend', () => {
                toast.remove();
            });
        }, durationMs);
    }
}