// Card stuff here. we're gonna need some stuff for this

export function CardHeader() {
    return (
        <div className="card-header">
            <h2 className="card-title">Card Title</h2>
        </div>
    );
}

export function CardContent() {
    return (
        <div className="card-content">
            <p>This is the card content.</p>
        </div>
    );
}

export function CardFooter() {
    return (
        <div className="card-footer">
            <button className="card-button">Action</button>
        </div>
    );
}

export function Card() {
    return (
        <div className="card">
        </div>
    );
}