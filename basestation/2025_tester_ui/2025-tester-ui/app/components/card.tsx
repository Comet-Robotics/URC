// Card stuff here. we're gonna need some stuff for this - JC

// Card header - JC

export function CardHeader() {
    return (
        <div className="card-header">
            <h2 className="card-title">Card Title</h2>
        </div>
    );
}

// Card content - JC

export function CardContent() {
    return (
        <div className="card-content">
            <p>This is the card content.</p>
        </div>
    );
}

// Card footer - JC

export function CardFooter() {
    return (
        <div className="card-footer">
            <button className="card-button">Action</button>
        </div>
    );
}

// Main Card component - JC

export function Card() {
    return (
        <div className="card">
            <CardHeader />
            <CardContent />
            <CardFooter />
        </div>
    );
}