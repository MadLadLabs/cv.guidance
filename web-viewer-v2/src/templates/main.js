class ReticleControl {
    constructor(overlayElementId, overlayContainerElementId, gateValueElementId, panZoomControl){
        this.overlayElementId = overlayElementId;
        this.overlayElement = null;
        this.overlayContainerElementId = overlayContainerElementId;
        this.overlayContainerElement = null;
        this.gateValueElementId = gateValueElementId;
        this.gateValueElement = null;
        this.overlayContext = null;
        this.panZoomControl = panZoomControl;
        this.reticle = {
            x: 0,
            y: 0,
            minX: 0,
            minY: 0,
            maxX: 1,
            maxY: 1
        };
        this.previousReticle = {
            x: 0, y: 0
        };
        this.sensitivity = 1/5000;
        this.gateLevels = [5,10,15,20,25];
        this.gateIndex = 0;
        this.gateChanged = true;
        this.panZoomControl.setReticleControl(this);
    }

    increaseGateSize() {
        if (this.gateIndex < this.gateLevels.length - 1)
        {
            this.gateIndex++;
            this.gateChanged = true;
            this.updateDOM();
        }
    }
    
    decreaseGateSize() {
        if (this.gateIndex > 0)
        {
            this.gateIndex--;
            this.gateChanged = true;
            this.updateDOM();
        }
    }

    updateReticle(dx, dy) {
        this.previousReticle.x = this.reticle.x;
        this.previousReticle.y = this.reticle.y;
        this.reticle.x += dx * this.sensitivity;
        this.reticle.y += -dy * this.sensitivity;
        var pan_dx = 0;
        var pan_dy = 0;
        var pan_passthrough = false;
        if (this.reticle.x < this.reticle.minX || this.reticle.x > this.reticle.maxX){
            pan_dx = dx;
            pan_passthrough = true;
        }
        if (this.reticle.y < this.reticle.minY || this.reticle.y > this.reticle.maxY){
            pan_dy = dy;
            pan_passthrough = true;
        }
        if (pan_passthrough) {
            this.panZoomControl.updatePan(pan_dx, pan_dy);
        }
        this.reticle.x = Math.max(this.reticle.x, this.reticle.minX);
        this.reticle.x = Math.min(this.reticle.x, this.reticle.maxX);
        this.reticle.y = Math.max(this.reticle.y, this.reticle.minY);
        this.reticle.y = Math.min(this.reticle.y, this.reticle.maxY);
        this.updateDOM();
    }

    calculateCanvasGateSize() {
        var imageNaturalWidth = this.panZoomControl.videoFeed.naturalWidth;
        var imageWidth = this.panZoomControl.videoFeed.getBoundingClientRect().width;
        return imageWidth / imageNaturalWidth * this.gateLevels[this.gateIndex];
    }

    updateDOM(force=false) {
        if (!this.overlayContext) {
            this.overlayElement = document.getElementById(this.overlayElementId);
            this.overlayContext = this.overlayElement.getContext('2d');
        }
        if (!this.overlayContainerElement) {
            this.overlayContainerElement = document.getElementById(this.overlayContainerElementId);
        }
        if (!this.gateValueElement) {
            this.gateValueElement = document.getElementById(this.gateValueElementId);
        }
        if (this.overlayContext &&
            this.overlayContainerElement &&
            this.gateValueElement &&
            (this.previousReticle.x != this.reticle.x || this.previousReticle.y != this.reticle.y || this.gateChanged || force)) {
            this.gateChanged = false;
            this.gateValueElement.innerText = `${this.gateLevels[this.gateIndex]}`;
            var overlayContainerElementDimensions = this.overlayContainerElement.getBoundingClientRect();
            if (this.overlayElement.width != overlayContainerElementDimensions.width || 
                this.overlayElement.height != overlayContainerElementDimensions.height) {
                    this.overlayElement.width = overlayContainerElementDimensions.width;
                    this.overlayElement.height = overlayContainerElementDimensions.height;
            }
            this.overlayContext.clearRect(0,0,this.overlayElement.width,this.overlayElement.height); //clear canvas
            this.overlayContext.beginPath();
            var canvasGateSize = this.calculateCanvasGateSize();
            var width = canvasGateSize;
            var height = canvasGateSize;
            var reticleX = this.overlayElement.width * this.reticle.x - width / 2;
            var reticleY = this.overlayElement.height * this.reticle.y - height / 2;
            this.overlayContext.rect(reticleX,reticleY,width,height);
            this.overlayContext.strokeStyle = 'green';
            this.overlayContext.lineWidth = 1;
            this.overlayContext.stroke();
        }
    }
}

class PanZoomControl {
    constructor(videoFeedElementId, videoFeedContainerElementId, zoomValueElementId){
        this.videoFeedElementId = videoFeedElementId;
        this.videoFeedContainerElementId = videoFeedContainerElementId;
        this.zoomValueElementId = zoomValueElementId;
        this.videoFeedContainer = null;
        this.videoFeed = null;
        this.zoomValue = null;
        this.pan = {
            x: 0,
            y: 0,
            minX: 0,
            minY: 0,
            maxX: 1,
            maxY: 1
        };
        this.previousPan = {
            x: 0, y: 0
        };
        this.sensitivity = 1/5000;
        this.zoomLevels = [1,2,3,4];
        this.zoomIndex = 0;
        this.previousZoomIndex = 0;
        this.zoomChanged = true;
        this.reticleControl = null;
    }

    setReticleControl(reticleControl){
        this.reticleControl = reticleControl;
    }

    zoomIn() {
        if (this.zoomIndex < this.zoomLevels.length - 1)
        {
            this.previousZoomIndex = this.zoomIndex;
            this.zoomIndex++;
            this.zoomChanged = true;
            this.updateDOM();
        }
    }
    
    zoomOut() {
        if (this.zoomIndex > 0)
        {
            this.previousZoomIndex = this.zoomIndex;
            this.zoomIndex--;
            this.zoomChanged = true;
            this.updateDOM();
        }
    }

    updatePan(dx, dy) {
        this.previousPan.x = this.pan.x;
        this.previousPan.y = this.pan.y;
        this.pan.x += dx * this.sensitivity;
        this.pan.y += -dy * this.sensitivity;
        this.pan.x = Math.max(this.pan.x, this.pan.minX);
        this.pan.x = Math.min(this.pan.x, this.pan.maxX);
        this.pan.y = Math.max(this.pan.y, this.pan.minY);
        this.pan.y = Math.min(this.pan.y, this.pan.maxY);
        this.updateDOM();
    }

    resetPan() {
        this.pan.x = 0;
        this.pan.y = 0;
        this.previousPan.x = 0;
        this.previousPan.y = 0;
    }

    updatePanAndReticleOnZoom() {
        var previousZoom = this.zoomLevels[this.previousZoomIndex];
        var newZoom = this.zoomLevels[this.zoomIndex];
        var newPanX = 0;
        var newPanY = this.pan.y;
        var newReticleX = 0;
        var newReticleY = 0;

        if (newZoom > 1){
            // calculate pan assuming the reticle can be in the middle of the viewport
            newPanX = 1/(newZoom - 1) * (newZoom/previousZoom*((previousZoom-1)*this.pan.x + this.reticleControl.reticle.x)-0.5);
            newPanY = 1/(newZoom - 1) * (newZoom/previousZoom*((previousZoom-1)*this.pan.y + this.reticleControl.reticle.y)-0.5);
        }

        if (newPanX <= 0) {
            newPanX = 0;
        }
        if (newPanX >= 1) {
            newPanX = 1;
        }
        if (newPanY <= 0) {
            newPanY = 0;
        }
        if (newPanY >= 1) {
            newPanY = 1;
        }
        newReticleX = newZoom/previousZoom*((previousZoom-1)*this.pan.x + this.reticleControl.reticle.x) - (newZoom-1)*newPanX;
        newReticleY = newZoom/previousZoom*((previousZoom-1)*this.pan.y + this.reticleControl.reticle.y) - (newZoom-1)*newPanY;

        this.pan.x = newPanX;
        this.pan.y = newPanY;
        this.reticleControl.reticle.x = newReticleX;
        this.reticleControl.reticle.y = newReticleY;

    }

    updateDOM() {
        if (!this.videoFeedContainer) {
            this.videoFeedContainer = document.getElementById(this.videoFeedContainerElementId);
        }
        if (!this.videoFeed) {
            this.videoFeed = document.getElementById(this.videoFeedElementId);
        }
        if (!this.zoomValue) {
            this.zoomValue = document.getElementById(this.zoomValueElementId);
        }
        if (this.videoFeed && 
            this.videoFeedContainer && 
            this.zoomValue &&
            (this.previousPan.x != this.pan.x || this.previousPan.y != this.pan.y || this.zoomChanged)) 
        {
            var zoom = this.zoomLevels[this.zoomIndex];
            if (this.zoomChanged){
                this.updatePanAndReticleOnZoom();
            }
            this.zoomValue.innerText = `${zoom}X`;
            this.videoFeed.style.width = `${zoom*100}%`;
            var videoFeedDimensions = this.videoFeed.getBoundingClientRect();
            var videoFeedContainerDimensions = this.videoFeedContainer.getBoundingClientRect();
            var panRangeY = videoFeedDimensions.height - videoFeedContainerDimensions.height;
            var panRangeX = videoFeedDimensions.width - videoFeedContainerDimensions.width;
            var yOffset = panRangeY * this.pan.y;
            var xOffset = panRangeX * this.pan.x;
            this.videoFeed.style.top = `-${yOffset}px`;
            this.videoFeed.style.left = `-${xOffset}px`;
            this.zoomChanged = false;
            this.reticleControl.updateDOM(true);
        }
    }
}

var panZoomControl = new PanZoomControl('video_feed', 'video_feed_container', 'zoom_value');

var reticleControl = new ReticleControl('canvas', 'overlay_container', 'gate_value', panZoomControl);

window.onload = function() {
    var panJoystick = new JoyStick('joystick_pan');
    var reticleJoystick = new JoyStick('joystick_reticle');

    setInterval(function () {
        var joystick_x = parseFloat(panJoystick.GetX());
        var joystick_y = parseFloat(panJoystick.GetY());
        panZoomControl.updatePan(joystick_x, joystick_y);
    }, 33);

    setInterval(function () {
        var joystick_x = parseFloat(reticleJoystick.GetX());
        var joystick_y = parseFloat(reticleJoystick.GetY());
        reticleControl.updateReticle(joystick_x, joystick_y);
    }, 33);
}
