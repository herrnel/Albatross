from core.types.module.module_type import Module

import threading

class Pipeline:
    """
    Purpose is to initialize, organize, store, and sychronize the pipeline, control, and message threads. 
    """
    modules: dict[str, Module]

    def __init__(self, modules: list[Module]):
        self.modules = {}

        for module in modules:
            self.modules[module.name] = module
            setattr(self, module.name, module)
        
    def take_control(self) -> None:
        ctrl_thread = threading.Thread(
            target=self.modules.control_module.control_loop,
            args=(self.shared_data, self.stop_evt, 500.0, "scripted"),
            daemon=True,
        )
        ctrl_thread.start()
        
    def start_processing(self) -> None:
        
            
            
    def send_init(self) -> None: 
        self.send_thread = threading.Thread(target=command_loop, args=(mav, shared, stop_evt, t0, 50.0), daemon=True)
    
    def pump_init(self) -> None: 
        self.pump_thread = threading.Thread(target=pump_loop, args=(mav, shared, stop_evt), daemon=True)
        
    def print_init(self) -> None: 
        self.print_thread = threading.Thread(target=hb_print_loop, args=(shared, stop_evt), daemon=True)
        
    def send_start(self) -> None: 
        self.send_thread.start()
        
    def pump_start(self) -> None: 
        self.pump_thread.start()
    
    def print_start(self) -> None: 
        self.print_thread.start()
        