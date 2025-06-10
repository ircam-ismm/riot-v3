mgraphics.init();
mgraphics.relative_coords = 0;
mgraphics.autofill = 0;
outlets = 1;

var numSegments = jsarguments.length > 1 ? jsarguments[1] : 10; // Number of segments in the bargraph
var isHorizontal = jsarguments.length > 2 ? jsarguments[2] : 0; // 0 = vertical, 1 = horizontal
var minValue = 3.4;
var maxValue = 4.2;
var currentValue = 0;
var soc = 0;

function paint() {
    var width = mgraphics.size[0];
    var height = mgraphics.size[1];
    var segmentSize = isHorizontal ? width / numSegments : height / numSegments;

    mgraphics.set_source_rgb(0.2, 0.2, 0.2);
    mgraphics.rectangle(0, 0, width, height);
    mgraphics.fill();
     
    var fillSegments = Math.round(soc * numSegments);
    
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
	soc = voltageToSoC(currentValue);
	outlet(0, soc);
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



function voltageToSoC(v) {
    var soc_map = [
        [4.20, 100], [4.15, 95], [4.10, 90], [4.00, 85],
        [3.90, 80], [3.85, 70], [3.80, 60], [3.75, 50],
        [3.70, 40], [3.65, 30], [3.60, 20], [3.55, 10],
        [3.50, 5],  [3.45, 2],  [3.40, 0]
    ];
	var soc_final;
    if (v >= soc_map[0][0]) return 1.0;
    if (v <= soc_map[soc_map.length - 1][0]) return 0.0;

    for (var i = 0; i < soc_map.length - 1; i++) {
        var v1 = soc_map[i][0], soc1 = soc_map[i][1];
        var v2 = soc_map[i+1][0], soc2 = soc_map[i+1][1];
        if (v <= v1 && v > v2) {
            var t = (v - v2) / (v1 - v2);
			soc_final = soc2 + t * (soc1 - soc2);
			soc_final = soc_final / 100.0;
            return soc_final;
        }
    }
	
    return 0.0;
}
