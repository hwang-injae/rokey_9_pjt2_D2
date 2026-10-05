import json

CONFIG = '/home/injae/rokey9_pjt2_hmi/config.json'


class ProbeLoader:
    def load(self):
        with open(CONFIG, encoding='utf-8') as f:
            return json.load(f)
