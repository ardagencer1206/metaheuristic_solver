from flask import Flask
from routes.alns_routes import alns_bp

def create_app():
    app = Flask(__name__)
    app.register_blueprint(alns_bp, url_prefix="/alns")
    return app

if __name__ == "__main__":
    app = create_app()
    app.run(host="0.0.0.0", port=8080)
