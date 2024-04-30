

function createTerminal(callback) {
    var container = document.querySelector('.log');
    var term = new Terminal({
        fontFamily: 'Courier New',  // Set the font family to 'Monospace'
        fontSize: 14,  // Set the font size to 14 pixels
        letterSpacing: 1,  // Set the spacing between letters to 0
        lineHeight: 1  // Set the height of each line to 1
    });

    // Open the terminal in the container
    term.open(container);

    term.write('Hello from xterm.js');

    // Call the callback function with the created terminal
    callback(term);
}

function resizeTerminal(term) {
    var screenWidth = window.innerWidth;
    var padding = 20;  // Padding and border 
    var gridGap = 10;  // Grid gap between the terminal and the window
    var streamerWidth = 640;  // Width of the image streamer in pixels
    var TerminalWidth = screenWidth - streamerWidth - padding - gridGap;
    console.log('Terminal width: ' + TerminalWidth + ' pixels');

    var defaultCharWidth = 9;  // Default width of a character in pixels
    var defaultCharHeight = 15;  // Default height of a character in pixels
    var cols = Math.floor(TerminalWidth / defaultCharWidth);
    var rows = Math.floor(430 / defaultCharHeight);
    console.log('Resizing terminal to ' + cols + ' cols and ' + rows + ' rows');
    term.resize(cols, rows);
}


// Usage:
document.addEventListener("DOMContentLoaded", function () {
    createTerminal(function (term) {
        resizeTerminal(term);

        var socketUrl = 'http://' + window.location.hostname + ':5000';
        var socket = io.connect(socketUrl );
        socket.on('new_message', function (data) {
            term.writeln(data.message);
            console.log(data.message);
        });

        // Listen for window resize events and resize the terminal accordingly
        window.addEventListener('resize', function () {
            resizeTerminal(term);

        });
    });
});

document.querySelectorAll('.button').forEach(function (button) {
    button.addEventListener('click', function () {
        var buttonName = button.getAttribute('data-button-name');
        fetch('/button_click/' + buttonName, { method: 'POST' })
            .then(function (response) {
                if (response.ok) {
                    console.log('Click was recorded');
                    return;
                }
                throw new Error('Request failed.');
            })
            .catch(function (error) {
                console.log(error);
            });
    });
});