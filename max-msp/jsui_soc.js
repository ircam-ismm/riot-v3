autowatch = 1;

// Parameters
var num_bars = jsarguments.length > 1 ? jsarguments[1] : 10; // Number of segments in the bargraph
var isHorizontal = jsarguments.length > 4 ? jsarguments[4] : 0; // 0 = vertical, 1 = horizontal
var cell_voltage = 3.7; // average cell voltage for 1-cell LiPo
var voltage = 4.2;      // measured input voltage
var cell_count = 1;     // number of cells in series


// Nonlinear voltage-to-SoC map (1 cell)
var soc_map = [
    [4.20, 100], [4.15, 95], [4.10, 90], [4.00, 85],
    [3.90, 80], [3.85, 70], [3.80, 60], [3.75, 50],
    [3.70, 40], [3.65, 30], [3.60, 20], [3.55, 10],
    [3.50, 5], [3.45, 2], [3.40, 0]
];

// Redraw when UI changes
function bang() {
    draw();
}

// Set voltage from Max
function msg_float(v) {
    voltage = v;
    draw();
}

// Allow setting number of bars
function bars(n) {
    num_bars = Math.max(1, n);
    draw();
}

// Set number of cells
function cells(n) {
    cell_count = Math.max(1, n);
    draw();
}

// Estimate SoC from voltage
function estimate_soc(v) {
    var per_cell_v = v / cell_count;
    if (per_cell_v >= soc_map[0][0]) return 100;
    if (per_cell_v <= soc_map[soc_map.length - 1][0]) return 0;

    for (var i = 0; i < soc_map.length - 1; i++) {
        var v1 = soc_map[i][0];
        var soc1 = soc_map[i][1];
        var v2 = soc_map[i + 1][0];
        var soc2 = soc_map[i + 1][1];

        if (per_cell_v <= v1 && per_cell_v > v2) {
            var t = (per_cell_v - v2) / (v1 - v2);
            return soc2 + t * (soc1 - soc2);
        }
    }
    return 0;
}

// Draw the bargraph
function draw() {
    var w = box.rect[2] - box.rect[0];
    var h = box.rect[3] - box.rect[1];

    var g = new Global("graphics");
    with (sketch) {
        reset();
        glclearcolor(0.15, 0.15, 0.15, 1);
        glclear();

        var soc = estimate_soc(voltage);
        var filled = Math.round((soc / 100) * num_bars);
        var margin = 2;
        var bar_width = (w - (num_bars + 1) * margin) / num_bars;
        var bar_height = h - 2 * margin;

        for (var i = 0; i < num_bars; i++) {
            var x = margin + i * (bar_width + margin);
            var y = margin;

            if (i < filled) {
                var t = i / (num_bars - 1); // blend 0–1
                var r = 1.0 - Math.min(t * 2, 1.0); // red → 0
                var g = Math.min(t * 2, 1.0);       // green → 1
                glcolor(r, g, 0);
            } else {
                glcolor(0.3, 0.3, 0.3); // gray for empty
            }

            moveto(x, y);
            rect(bar_width, bar_height);
        }

        refresh();
    }
}

function set_orientation(horizontal) {
    isHorizontal = horizontal ? 1 : 0;
    bang();
}