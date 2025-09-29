from fastapi import FastAPI, HTTPException, Query, Depends
from sqlalchemy import Column, Float, DateTime, create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, Session
from typing import Optional
import math
from fastapi.middleware.cors import CORSMiddleware

# --- FastAPI app (only once!) ---
app = FastAPI()

# --- CORS middleware ---
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://127.0.0.1:5500", "http://localhost:5500"],  # frontend origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- Database setup (SQL Server, pyodbc) ---
DATABASE_URL = (
    "mssql+pyodbc://@FELIX-NGUYEN\\SQLEXPRESS/final_project_microcontroller"
    "?driver=ODBC Driver 17 for SQL Server&Trusted_Connection=yes"
)

engine = create_engine(DATABASE_URL, pool_pre_ping=True)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

# --- SQLAlchemy model (match your real table: no id column) ---
class Temperature(Base):
    __tablename__ = "temperature"
    reading_time = Column(DateTime, primary_key=True, nullable=False)
    temperature_value = Column(Float, nullable=False)

# Dependency: get DB session
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


# ✅ GET latest temperature
@app.get("/api/temperature/latest")
def get_latest_temperature(db: Session = Depends(get_db)):
    latest = (
        db.query(Temperature)
        .order_by(Temperature.reading_time.desc())
        .first()
    )

    if not latest:
        raise HTTPException(status_code=404, detail="No temperature readings found.")

    return {
        "reading_time": latest.reading_time,
        "temp_c": latest.temperature_value,
        "temp_f": latest.temperature_value * 9 / 5 + 32,
    }


# ✅ GET temperature list with filters + pagination
@app.get("/api/temperature")
def get_temperatures(
    start_time: Optional[str] = Query(None),
    end_time: Optional[str] = Query(None),
    page: int = Query(1, gt=0),
    page_size: int = Query(10, gt=0),
    db: Session = Depends(get_db),
):
    query = db.query(Temperature)

    # Apply filters
    if start_time and not end_time:
        query = query.filter(Temperature.reading_time == start_time)

    if end_time and not start_time:
        query = query.filter(Temperature.reading_time == end_time)

    if start_time and end_time:
        query = query.filter(
            Temperature.reading_time >= start_time,
            Temperature.reading_time <= end_time,
        )

    # Pagination
    total_items = query.count()
    total_pages = math.ceil(total_items / page_size) if total_items else 0

    if page > total_pages and total_items > 0:
        raise HTTPException(
            status_code=404,
            detail=f"Page {page} does not exist. Total pages: {total_pages}.",
        )

    rows = (
        query.order_by(Temperature.reading_time.desc())
        .offset((page - 1) * page_size)
        .limit(page_size)
        .all()
    )

    return {
        "total_items": total_items,
        "total_pages": total_pages,
        "current_page": page,
        "page_size": page_size,
        "data": [
            {
                "reading_time": r.reading_time,
                "temperature_value": r.temperature_value,
            }
            for r in rows
        ],
    }
