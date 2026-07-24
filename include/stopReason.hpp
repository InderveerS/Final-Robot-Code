#pragma once

// Why a blocking "...Until" primitive returned.
//   Event   - the caller's event predicate fired
//   Limit   - the built-in backstop was reached (distance for move/follow,
//             angle for turn)
//   Timeout - neither happened before timeoutMs elapsed
enum class StopReason { Event, Limit, Timeout };
