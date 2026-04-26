/**
 * API client wrapper.
 *
 * Re-export'ит auto-generated services и models из ./generated/.
 * Generated файлы создаются командой:
 *
 *   npm run generate:api
 *
 * (npm script: сначала зовёт generate:openapi → пишет openapi.json,
 *  потом openapi-typescript-codegen генерит TS клиент в ./generated/)
 *
 * Конвенции:
 *   - Имена method'ов в Services длинные (typeName + path), напр.
 *     `RobotService.setVelocityApiV1RobotVelocityPost(...)`. Это не очень
 *     удобно — поэтому ниже re-export'им Services под короткими алиасами.
 *
 *   - Старый `src/lib/api.ts` (legacy `api.X()`) пока продолжает работать
 *     поверх deprecated-alias middleware'а на бэке. Новые компоненты
 *     должны импортить из этого модуля. lib/api.ts будет удалён в #6.
 */

// Core (runtime exports — без `type`)
export {
  ApiError,
  CancelablePromise,
  CancelError,
  OpenAPI,
} from './generated';
export type { OpenAPIConfig } from './generated';

// Services
export {
  ActuatorsService,
  CameraService,
  ControlService,
  DefaultService,
  DetectionService,
  FsmService,
  MapsService,
  RobotService,
  SamcanService,
  SensorsService,
  SystemService,
} from './generated';

// Models — все `export type` (тип-only re-export). Embedded типы
// (RobotPose, RobotStatus, LedState, RobotInfo, MultiRobotListResponse)
// codegen не вынес отдельно — они доступны inline в response model'ях.
export type {
  ActuatorsResponse,
  ArmJointCommand,
  ArmResponse,
  ArmState,
  BallInfo,
  BallsResponse,
  BatteryResponse,
  CalibrationCoefficientsResponse,
  CalibrationCommand,
  CalibrationProfile,
  CalibrationProfileDeleteCommand,
  CalibrationProfileListResponse,
  CalibrationProfileLoadCommand,
  CalibrationProfileSaveCommand,
  CalibrationSetCommand,
  CameraEndpointResponse,
  ClawCommand,
  ClawResponse,
  ClawState,
  ClosestDetectionResponse,
  CommandAck,
  Detection,
  DetectionResponse,
  DetectionStatusResponse,
  DetectionToggleCommand,
  ExplorerCommand,
  FollowMeCommand,
  ForbiddenZone,
  FsmCommand,
  FsmStateResponse,
  FsmTransitionCommand,
  HardwareActiveResponse,
  HardwareApplyCommand,
  HardwarePresetListResponse,
  HardwarePresetResponse,
  HardwarePresetSaveCommand,
  HardwarePresetSummary,
  HeadCommand,
  HeadResponse,
  HeadState,
  HTTPValidationError,
  ImuData,
  ImuResponse,
  ImuYpr,
  LedCommand,
  LogEntry,
  LogResponse,
  MapInfo,
  MapInfoResponse,
  MapListResponse,
  MapLoadCommand,
  MapSaveCommand,
  MissionCommand,
  MissionListResponse,
  MqttStatusResponse,
  MultiRobotCallCommand,
  MultiRobotInfo,
  MultiRobotListResponseSystem,
  PathListResponse,
  PathRecorderCommand,
  PathRecorderPathResponse,
  PatrolCommand,
  PatrolWaypoint,
  PatrolWaypointsCommand,
  PlannedPathResponse,
  PoseResponse,
  PrecisionDriveCommand,
  PresetInfo,
  PresetListResponse,
  PresetLoadCommand,
  PresetSaveCommand,
  SensorsResponse,
  SlamMapResponse,
  SlamObject,
  SpeedProfileCommand,
  SpeedProfileResponse,
  StatusResponse,
  StatusSnapshotResponse,
  TemperatureResponse,
  ToggleCommand,
  TTSSpeakCommand,
  TTSToggleCommand,
  UltrasonicData,
  UltrasonicResponse,
  ValidationError,
  Vec3,
  VelocityCommand,
  VelocityDetail,
  VelocityResponse,
  ZoneCreateCommand,
  ZoneCreatedResponse,
  ZonesResponse,
} from './generated';
