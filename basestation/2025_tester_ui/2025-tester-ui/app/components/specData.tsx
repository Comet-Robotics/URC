// Component in the SCIENCE PAYLOAD page
// This component gets both kinds of spec data

// Function that takes raman spec data and makes it displayable

let ramanString = "";

function ramanSpecData(stringData = ""){

    if (stringData == ""){
        stringData = "Analyzing...";
    }

    return stringData;
}

// Function that takes vis spec data and makes it displayable

let visString = "";

function visSpecData(stringData = ""){

    if (stringData == ""){
        stringData = "Analyzing...";
    }

    return stringData;
}


// Actual component

export function SpecData(){
    return(
        <div className="specData">
            <h3>Ramen Spec Data</h3>
            <p>Current Scan: {ramanSpecData(ramanString)}</p>
            <hr></hr>
            <h3>Vis Spec Data</h3>
            <p>Current Scan: {visSpecData(visString)}</p>
        </div>
    )
}