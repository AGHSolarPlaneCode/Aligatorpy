from Application.Services.MatekService import MatekService

drone = MatekService(device="tcp:192.168.x.x:5760")

drone.send_landing_sites([
    (50.1, 19.1),
    (50.2, 19.2),
    (50.3, 19.3),
    (50.4, 19.4),
])

print("Przesłano lądowiska pomyślnie")