import React from 'react';
function VoyagePageTemplate({ image, header, desc }) {
    const containerStyle = {
        display: 'flex',
        flexDirection: 'column',
        backgroundColor: '#3498db',
        color: 'white',
        fontFamily: 'Verdana, Geneva, sans-serif',
        width: '100%',
        height: '100%',
        boxSizing: 'border-box',
        position: 'relative',
    };

    const headerStyle = {
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        padding: '20px',
        width: 'auto',
        backgroundColor: '#2c3e50',
        minHeight: '300px',
        position: 'relative'
    };

    const imageStyle = {
        width: '40%',
        height: '200%',
        position: 'absolute',
        top: '20%',
        left: '25%',
        transform: 'translateX(-50%)',
    };

    const headerTextStyle = {
        fontSize: '2.5em',
        fontWeight: 'bold',
        margin: 1,
        zIndex: 1,
        position: 'relative',
        right: '-10%'
    };

    const descBoxStyle = {
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        padding: '20px',
        backgroundColor: '#ecf0f1',
        color: '#2c3e50',
        fontSize: '1em',
        width: '65%',
        boxSizing: 'border-box',
        marginTop: '20px',
        marginLeft: 'auto',
        marginRight: 'auto',
    };

    const descStyle = {
        textAlign: 'center',
        width: '100%',
        fontSize: '0.9em',
        wordBreak: 'break-word',
    };
    return (

        <div style={containerStyle}>
            <div style={headerStyle}>
                <img src='Polaris.png' alt="Voyage" style={imageStyle} />
                <h1 style={headerTextStyle}>{header}</h1>
            </div>
            <div style={descBoxStyle}>
                <p style={descStyle}>{desc}</p>
            </div>
        </div>
    );
}

export default VoyagePageTemplate;
