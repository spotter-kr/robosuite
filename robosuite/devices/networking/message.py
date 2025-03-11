from typing import Annotated, List, Optional
from annotated_types import Len
from enum import Enum
from pydantic import BaseModel

class PoseMessage(BaseModel):
    pose: Annotated[List[float], Len(min_length=16, max_length=16)]
    gripperOpenAmount: float

class HomePoseMessage(BaseModel):
    unused: Optional[int | None] = None # dummy field because Pydantic doesn't support empty models