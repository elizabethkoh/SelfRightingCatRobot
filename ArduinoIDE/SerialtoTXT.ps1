$filePath = "C:\Users\Elizabeth\FrontierLab\ArduinoIDE\data.txt"
$serialPort = New-Object System.IO.Ports.SerialPort "COM16", 115200, None, 8, one
$serialPort.Open()
do {
    $line = $serialPort.ReadLine()
    Add-Content -Path $filePath -Value $line
} while ($serialPort.IsOpen)