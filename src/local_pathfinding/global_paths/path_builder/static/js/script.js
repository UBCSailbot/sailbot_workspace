var MAP_ZOOM = 6;
// Initialize the map
var map = L.map('map').setView([49.1536, -125.9067], MAP_ZOOM);

// Load a tile layer
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '© OpenStreetMap contributors'
}).addTo(map);

var waypoints = [];

// Event listener for clicking on the map
map.on('click', function(e) {
    // Get coordinates of the clicked point
    var lat = e.latlng.lat.toFixed(MAP_ZOOM);
    var lon = e.latlng.lng.toFixed(MAP_ZOOM);

    // Add coordinates to the waypoints array
    waypoints.push({lat,lon});
    refresh();
});

function draw_marker(item){
    L.marker([item.lat, item.lon]).addTo(map).bindPopup(`Waypoint: ${item.lat}, ${item.lon}`);
}

function draw_polyline(item, index){
    if (index > 0){
        var prevLatLon = waypoints[index - 1];
        L.polyline([[prevLatLon.lat, prevLatLon.lon], [item.lat, item.lon]]).addTo(map);
    }
}

function refresh(){

    // clear map and redraw markers and polylines
    map.eachLayer(function(layer) {
        if (layer instanceof L.Marker || layer instanceof L.Polyline) {
            layer.remove();
        }
    });

    waypoints.forEach(draw_marker);

    if (waypoints.length > 1) {
        waypoints.forEach(draw_polyline);
    }

    update_waypoints_table();
}
// Button event handlers
function clear_path(){

    var confirmation = confirm("Are you sure you want to clear all waypoints? You cannot undo this action.");

    if (confirmation) {
        waypoints = [];
        refresh();
    }
}
function import_file(){

    confirmation = true;

    if (waypoints.length > 0) {

        var confirmation = confirm("Be sure you have exported your current path before importing a new one. Are you sure you want to continue?");
    }

    if (confirmation) {
        waypoints = [];
        refresh();

        var filename = window.prompt("Enter the filename of the CSV file to import:", "");

        if (filename === null) {
            // User clicked Cancel, do nothing
            return;
        }

        else if (filename === "") {
            alert("You must enter a filename.");
            return;
        }

        fetch('/import_waypoints', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ filename: filename }),
        })
        .then(response => response.json())
        .then(data => {
            if (data.status === 'success'){
                waypoints = data.waypoints;
                refresh();
            } else {
                alert('Error importing waypoints. Please check the server logs for details.');
            }
        })
    }


}
function interpolate(){
        var confirmation = confirm("Are you sure you want to interpolate the path? You cannot undo this action.");

        if (confirmation) {
            var interval_spacing = window.prompt("Enter the desired interval spacing in km:", "30");
            fetch('/interpolate_path', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ waypoints: waypoints, interval_spacing: interval_spacing }),
            })
            .then(response => response.json())
            .then(data => {
                if (data.status === 'success'){
                    waypoints = data.waypoints;
                    refresh();
                } else {
                    alert('Error interpolating waypoints. Please check the server logs for details.');
                }
            })
            .catch(error => {
                console.error('Error:', error);
            });
        }
}
function delete_paths(){

    var key = window.prompt("Enter the keyword in the filenames you want to delete:", "test");

    if (key === null) {
        // User clicked Cancel, do nothing
        return;
    }

    var confirmation = confirm(`Are you sure you want to delete all paths containing the keyword \"${key}\"?
    All timestamped paths will also be deleted.`);

    if (confirmation) {

        fetch('/delete_paths', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ key: key }),
        })
        .then(response => response.json())
        .then(data => {
            if (data.status === 'error'){
                alert('Error deleting paths. Please check the server logs for details.');
            }
            else{
                alert(`\"${key}\" Paths deleted successfully.`);
            }
        })
        .catch(error => {
            console.error('Error:', error);
        });
    }
}

function plot(){
    if (waypoints.length < 2) {
        alert('Please add at least two waypoints to plot a path.');
        return;
    }
    else{
        fetch('/plot_path', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ waypoints: waypoints }),
        })
        .then(response => response.json())
        .then(data => {
            if (data.status === 'error'){
                alert('Error plotting waypoints. Please check the server logs for details.');
            }
        })
        .catch(error => {
            console.error('Error:', error);
        });
    }
}


function prompt_and_export() {
    if (waypoints.length < 2) {
        alert('Please add at least two waypoints to export.');
        return;
    }
    // Prompt user for filename
    var filename = window.prompt("Enter the desired filename for the CSV file:", "");

    if (filename === null) {
        // User clicked Cancel, do nothing
        return;
    }

    if (filename === "") {
        alert("You must enter a filename.");
        return;
    }

    fetch('/export_waypoints', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ filename: filename, waypoints: waypoints }),
    })
    .then(response => response.json())
    .then(data => {
        if (data.status === 'success'){
            alert('Waypoints exported successfully.');
        } else {
            alert('Error exporting waypoints. Please check the server logs for details.');
        }
    })
    .catch(error => {
        console.error('Error:', error);
    });
}

function delete_waypoint(index) {
    // Remove the waypoint from the waypoints array
    waypoints.splice(index, 1);

    refresh();
}

function edit_waypoint(index) {
    var lat = window.prompt("Enter the new latitude:", waypoints[index].lat);
    var lon = window.prompt("Enter the new longitude:", waypoints[index].lon);

    if (lat === null || lon === null) {
        // User clicked Cancel, do nothing
        return;
    }

    if (lat === "" || lon === "") {
        alert("You must enter a latitude and longitude.");
        return;
    }

    waypoints[index].lat = lat;
    waypoints[index].lon = lon;

    refresh();
}

function update_waypoints_table() {
    var tableBody = document.getElementById("waypointsTable").getElementsByTagName('tbody')[0];

    // Clear existing rows
    tableBody.innerHTML = '';

    // Add waypoints to the table
    waypoints.forEach(function (waypoint, index) {
        var newRow = tableBody.insertRow(tableBody.rows.length);

        var cell1 = newRow.insertCell(0);
        var cell2 = newRow.insertCell(1);
        var cell3 = newRow.insertCell(2);

        cell1.innerHTML = waypoint.lat;
        cell2.innerHTML = waypoint.lon;
        cell3.innerHTML = `<button type="button" class="btn" onclick="delete_waypoint(${index})"><i class="fa fa-trash"></i></button>`;
        cell3.innerHTML += `<button type="button" class="btn" onclick="edit_waypoint(${index})"><i class="fa fa-pencil"></i></button>`;
        cell3.style.display = "flex";
    });
}
