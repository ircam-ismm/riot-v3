
// Takes {0;1} input range

mgraphics.init();
mgraphics.relative_coords = 0;
mgraphics.autofill = 0;
outlets = 1;

var numSegments = jsarguments.length > 1 ? jsarguments[1] : 10; // Number of segments in the bargraph
var isHorizontal = jsarguments.length > 4 ? jsarguments[4] : 0; // 0 = vertical, 1 = horizontal
var minValue = 0.0;
var maxValue = 1.0;
var currentValue = 0;


function paint() {
    var width = mgraphics.size[0];
    var height = mgraphics.size[1];
    var segmentSize = isHorizontal ? width / numSegments : height / numSegments;

    mgraphics.set_source_rgb(0.2, 0.2, 0.2);
    mgraphics.rectangle(0, 0, width, height);
    mgraphics.fill();
    
    var fillSegments = Math.round((currentValue - minValue) / (maxValue - minValue) * numSegments);
    
    for (var i = 0; i < numSegments; i++) {
        if (i < fillSegments) {
            var fraction = i / numSegments;
            if (fraction < 0.5) {
                mgraphics.set_source_rgb(1, fraction * 2, 0); // Red to Yellow
            } else if (fraction < 0.75) {
                mgraphics.set_source_rgb(1 - (fraction - 0.5) * 2, 1, 0); // Yellow to Green
            } else {
                mgraphics.set_source_rgb(0, 1, 0); // Green
            }
        } else {
            mgraphics.set_source_rgb(0.5, 0.5, 0.5); // Inactive segment color
        }
        
        if (isHorizontal) {
            mgraphics.rectangle(i * segmentSize, 0, segmentSize - 2, height);
        } else {
            mgraphics.rectangle(0, height - (i + 1) * segmentSize, width, segmentSize - 2);
        }
        mgraphics.fill();
    }
}

function bang() {
    mgraphics.redraw();
}

function msg_float(value) {
    currentValue = Math.max(minValue, Math.min(maxValue, value));
	outlet(0, currentValue);
    bang();
}

function set_scale(min, max) {
    minValue = min;
    maxValue = max;
    bang();
}

function set_segments(count) {
    numSegments = Math.max(1, count);
    bang();
}

function set_orientation(horizontal) {
    isHorizontal = horizontal ? 1 : 0;
    bang();
}
