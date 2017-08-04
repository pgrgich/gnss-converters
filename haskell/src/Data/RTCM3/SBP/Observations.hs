{-# LANGUAGE NoImplicitPrelude #-}

-- |
-- Module:      Data.RTCM3.SBP.Observations
-- Copyright:   Copyright (C) 2016 Swift Navigation, Inc.
-- License:     LGPL-3
-- Maintainer:  Swift Navigation <dev@swiftnav.com>
-- Stability:   experimental
-- Portability: portable
--
-- RTCMv3 to SBP Observation Conversions.

module Data.RTCM3.SBP.Observations
  ( toSBPMsgObs
  ) where

import BasicPrelude
import Control.Lens
import Data.Bits
import Data.Int
import Data.IORef
import Data.List.Extra
import Data.RTCM3
import Data.RTCM3.SBP.Time
import Data.RTCM3.SBP.Types
import Data.Word
import SwiftNav.SBP

-- | Monadic IORef modify.
--
modifyIORefM :: MonadIO m => IORef a -> (a -> m (a, b)) -> m b
modifyIORefM ref f = do
  x      <- liftIO $ readIORef ref
  (y, z) <- f x
  liftIO $ writeIORef ref y
  return z

-- | Update and convert stored and incoming GPS times.
--
toGpsTimeNano :: MonadStore e m => Word16 -> (GpsTimeNano -> GpsTimeNano) -> m GpsTimeNano
toGpsTimeNano station rollover = do
  gpsTimeMap <- view storeGpsTimeMap
  modifyIORefM gpsTimeMap $ \gpsTimeMap' -> do
    t <- rollover <$> maybe currentGpsTime return (gpsTimeMap' ^. at station)
    return (gpsTimeMap' & at station ?~ t, t)

-- | Default observation doppler.
--
obsDoppler :: Doppler
obsDoppler = Doppler 0 0

-- | Default observation flags.
--
obsFlags :: Word8
obsFlags = 7

-- | Calculate pseudorange.
--
pseudorange :: Double -> Word32 -> Word8 -> Double
pseudorange u p amb = 0.02 * fromIntegral p + u * fromIntegral amb

-- | Calculate pseudorange difference.
--
pseudorangeDifference :: Int16 -> Double
pseudorangeDifference d = 0.02 * fromIntegral d

-- | GPS pseudorange unit.
--
gpsPseudorange :: Double
gpsPseudorange = 299792.458

-- | GLONASS pseudorange unit.
--
glonassPseudorange :: Double
glonassPseudorange = 599584.916

-- | Convert to SBP pseudorange.
--
toP :: Double -> Word32
toP p = round $ 50 * p

-- | Calculate carrier-phase.
--
carrierPhase :: Double -> Double -> Int32 -> Double
carrierPhase u p d = (p + 0.0005 * fromIntegral d) / (299792458.0 / u)

-- | GPS L1 carrier-phase unit.
--
gpsL1CarrierPhase :: Double
gpsL1CarrierPhase = 1.57542e9

-- | GPS L2 carrier-phase unit.
--
gpsL2CarrierPhase :: Double
gpsL2CarrierPhase = 1.22760e9

-- | GLONASS L1 carrier-phase unit.
--
glonassL1CarrierPhase :: Word8 -> Double
glonassL1CarrierPhase fcn = 1.602e9 + fromIntegral fcn * 0.5625e6

-- | GLONASS L2 carrier-phase unit.
--
glonassL2CarrierPhase :: Word8 -> Double
glonassL2CarrierPhase fcn = 1.246e9 + fromIntegral fcn * 0.4375e6

-- | Convert to SBP carrier-phase measurement.
--
toL :: Double -> CarrierPhase
toL l = if f /= 256 then CarrierPhase i (fromIntegral f) else CarrierPhase (i + 1) 0
  where
    i = floor l
    f = (round $ (l - fromIntegral i) * 256) :: Word16

-- | Convert from RTCMv3 lock indicator.
--
lock :: Word8 -> Word32
lock t
  |   t <  24 = 1000 * fromIntegral t
  |   t <  48 = 1000 * fromIntegral t *  2 -   24
  |   t <  72 = 1000 * fromIntegral t *  4 -  120
  |   t <  96 = 1000 * fromIntegral t *  8 -  408
  |   t < 120 = 1000 * fromIntegral t * 16 - 1176
  |   t < 127 = 1000 * fromIntegral t * 32 - 3096
  | otherwise = 1000 * 937

-- | Convert to SBP lock time.
--
toLock :: Word32 -> Word8
toLock t
  | t <     32 = 0
  | t <     64 = 1
  | t <    128 = 2
  | t <    256 = 3
  | t <    512 = 4
  | t <   1024 = 5
  | t <   2048 = 6
  | t <   4096 = 7
  | t <   8192 = 8
  | t <  16384 = 9
  | t <  32768 = 10
  | t <  65536 = 11
  | t < 131072 = 12
  | t < 262144 = 13
  | t < 524288 = 14
  |  otherwise = 15

-- | Map GPS L1 signal.
--
gpsL1Signal :: Word8 -> Bool -> GnssSignal16
gpsL1Signal sat code
  | code      = GnssSignal16 sat 5
  | otherwise = GnssSignal16 sat 0

-- | Map GPS L2 signal.
--
gpsL2Signal :: Word8 -> Word8 -> GnssSignal16
gpsL2Signal sat code
  | code == 0 = GnssSignal16 sat 1
  | otherwise = GnssSignal16 sat 6

-- | Map GLONASS L1 signal.
--
glonassL1Signal :: Word8 -> Bool -> Maybe GnssSignal16
glonassL1Signal sat code
  | code      = Nothing
  | otherwise = Just $ GnssSignal16 sat 3

-- | Map GLONASS L2 signal.
--
glonassL2Signal :: Word8 -> Word8 -> Maybe GnssSignal16
glonassL2Signal sat code
  | code == 0 = Just $ GnssSignal16 sat 4
  | otherwise = Nothing

-- | Max GPS satellite number.
--
gpsMaxSat :: Word8
gpsMaxSat = 32

-- | Max GLONASS satellite number.
--
glonassMaxSat :: Word8
glonassMaxSat = 24

-- | Produce SBP packed observation from GPS L1 observation.
--
toGpsL1PackedObsContents :: Word8 -> GpsL1Observation -> GpsL1ExtObservation -> Maybe PackedObsContent
toGpsL1PackedObsContents sat l1o l1eo
  | no        = Nothing
  | otherwise = Just PackedObsContent
      { _packedObsContent_P     = toP p1
      , _packedObsContent_L     = toL l1
      , _packedObsContent_D     = obsDoppler
      , _packedObsContent_cn0   = l1eo ^. gpsL1ExtObservation_cnr
      , _packedObsContent_lock  = toLock $ lock (l1o ^. gpsL1Observation_lockTime)
      , _packedObsContent_sid   = gpsL1Signal sat (l1o ^. gpsL1Observation_code)
      , _packedObsContent_flags = obsFlags
      }
  where
    no = sat > gpsMaxSat || l1o ^. gpsL1Observation_pseudorange == 524288 || l1o ^. gpsL1Observation_carrierMinusCode == -524288
    p1 = pseudorange gpsPseudorange (l1o ^. gpsL1Observation_pseudorange) (l1eo ^. gpsL1ExtObservation_ambiguity)
    l1 = carrierPhase gpsL1CarrierPhase p1 (l1o ^. gpsL1Observation_carrierMinusCode)

-- | Produce SBP packed observation from GPS L1 + L2 observation.
--
toGpsL2PackedObsContents :: Word8 -> GpsL1Observation -> GpsL1ExtObservation -> GpsL2Observation -> GpsL2ExtObservation -> Maybe PackedObsContent
toGpsL2PackedObsContents sat l1o l1eo l2o l2eo
  | no        = Nothing
  | otherwise = Just PackedObsContent
      { _packedObsContent_P     = toP p2
      , _packedObsContent_L     = toL l2
      , _packedObsContent_D     = obsDoppler
      , _packedObsContent_cn0   = l2eo ^. gpsL2ExtObservation_cnr
      , _packedObsContent_lock  = toLock $ lock (l2o ^. gpsL2Observation_lockTime)
      , _packedObsContent_sid   = gpsL2Signal sat (l2o ^. gpsL2Observation_code)
      , _packedObsContent_flags = obsFlags
      }
  where
    no = sat > gpsMaxSat || l2o ^. gpsL2Observation_pseudorangeDifference == -8192 || l2o ^. gpsL2Observation_carrierMinusCode == -524288
    p1 = pseudorange gpsPseudorange (l1o ^. gpsL1Observation_pseudorange) (l1eo ^. gpsL1ExtObservation_ambiguity)
    p2 = p1 + pseudorangeDifference (l2o ^. gpsL2Observation_pseudorangeDifference)
    l2 = carrierPhase gpsL2CarrierPhase p1 (l2o ^. gpsL2Observation_carrierMinusCode)

-- | Produce SBP packed observation from GLONASS L1 observation.
--
toGlonassL1PackedObsContents :: Word8 -> GlonassL1Observation -> GlonassL1ExtObservation -> Maybe PackedObsContent
toGlonassL1PackedObsContents sat l1o l1eo
  | no        = Nothing
  | otherwise = do
      sid <- glonassL1Signal sat (l1o ^. glonassL1Observation_code)
      Just PackedObsContent
        { _packedObsContent_P     = toP p1
        , _packedObsContent_L     = toL l1
        , _packedObsContent_D     = obsDoppler
        , _packedObsContent_cn0   = l1eo ^. glonassL1ExtObservation_cnr
        , _packedObsContent_lock  = toLock $ lock (l1o ^. glonassL1Observation_lockTime)
        , _packedObsContent_sid   = sid
        , _packedObsContent_flags = obsFlags
        }
  where
    no = sat > glonassMaxSat || l1o ^. glonassL1Observation_carrierMinusCode == -524288
    p1 = pseudorange glonassPseudorange (l1o ^. glonassL1Observation_pseudorange) (l1eo ^. glonassL1ExtObservation_ambiguity)
    l1 = carrierPhase (glonassL1CarrierPhase (l1o ^. glonassL1Observation_frequency)) p1 (l1o ^. glonassL1Observation_carrierMinusCode)

-- | Produce SBP packed observation from GLONASS L1 + L2 observation.
--
toGlonassL2PackedObsContents :: Word8 -> GlonassL1Observation -> GlonassL1ExtObservation -> GlonassL2Observation -> GlonassL2ExtObservation -> Maybe PackedObsContent
toGlonassL2PackedObsContents sat l1o l1eo l2o l2eo
  | no        = Nothing
  | otherwise = do
      sid <- glonassL2Signal sat (l2o ^. glonassL2Observation_code)
      Just PackedObsContent
        { _packedObsContent_P     = toP p2
        , _packedObsContent_L     = toL l2
        , _packedObsContent_D     = obsDoppler
        , _packedObsContent_cn0   = l2eo ^. glonassL2ExtObservation_cnr
        , _packedObsContent_lock  = toLock $ lock (l2o ^. glonassL2Observation_lockTime)
        , _packedObsContent_sid   = sid
        , _packedObsContent_flags = obsFlags
        }
  where
    no = sat > glonassMaxSat || l2o ^. glonassL2Observation_pseudorangeDifference == -8192 || l2o ^. glonassL2Observation_carrierMinusCode == -524288
    p1 = pseudorange glonassPseudorange (l1o ^. glonassL1Observation_pseudorange) (l1eo ^. glonassL1ExtObservation_ambiguity)
    p2 = p1 + pseudorangeDifference (l2o ^. glonassL2Observation_pseudorangeDifference)
    l2 = carrierPhase (glonassL2CarrierPhase (l1o ^. glonassL1Observation_frequency)) p1 (l2o ^. glonassL2Observation_carrierMinusCode)

-- | FromObservation produces L1 and L2 SBP packed observation(s).
--
class FromObservation a where
  l1PackedObsContents :: a -> Maybe PackedObsContent
  l2PackedObsContents :: a -> Maybe PackedObsContent

instance FromObservation Observation1002 where
  l1PackedObsContents o = toGpsL1PackedObsContents (o ^. observation1002_sat) (o ^. observation1002_l1) (o ^. observation1002_l1e)
  l2PackedObsContents _o = Nothing

instance FromObservation Observation1004 where
  l1PackedObsContents o = toGpsL1PackedObsContents (o ^. observation1004_sat) (o ^. observation1004_l1) (o ^. observation1004_l1e)
  l2PackedObsContents o = toGpsL2PackedObsContents (o ^. observation1004_sat) (o ^. observation1004_l1) (o ^. observation1004_l1e) (o ^. observation1004_l2) (o ^. observation1004_l2e)

instance FromObservation Observation1010 where
  l1PackedObsContents o = toGlonassL1PackedObsContents (o ^. observation1010_sat) (o ^. observation1010_l1) (o ^. observation1010_l1e)
  l2PackedObsContents _o = Nothing

instance FromObservation Observation1012 where
  l1PackedObsContents o = toGlonassL1PackedObsContents (o ^. observation1012_sat) (o ^. observation1012_l1) (o ^. observation1012_l1e)
  l2PackedObsContents o = toGlonassL2PackedObsContents (o ^. observation1012_sat) (o ^. observation1012_l1) (o ^. observation1012_l1e) (o ^. observation1012_l2) (o ^. observation1012_l2e)

-- | Convert RTCMv3 observation to SBP packed observation(s).
--
toPackedObsContent :: FromObservation a => [a] -> [PackedObsContent]
toPackedObsContent = concatMap $ (<>) <$> maybeToList . l1PackedObsContents <*> maybeToList . l2PackedObsContents

-- | Derive sender from station.
--
toSender :: Word16 -> Word16
toSender station = station .|. 61440

-- | FromObservations produces GPS time, SBP packed observations, and sender from RTCMv3 observation messages.
--
class FromObservations a where
  gpsTimeNano       :: MonadStore e m => a -> m GpsTimeNano
  packedObsContents :: a -> [PackedObsContent]
  sender            :: a -> Word16

instance FromObservations Msg1002 where
  gpsTimeNano m     = toGpsTimeNano (m ^. msg1002_header . gpsObservationHeader_station) $ rolloverTowGpsTime (m ^. msg1002_header . gpsObservationHeader_tow)
  packedObsContents = toPackedObsContent . view msg1002_observations
  sender            = toSender . view (msg1002_header . gpsObservationHeader_station)

instance FromObservations Msg1004 where
  gpsTimeNano m     = toGpsTimeNano (m ^. msg1004_header . gpsObservationHeader_station) $ rolloverTowGpsTime (m ^. msg1004_header . gpsObservationHeader_tow)
  packedObsContents = toPackedObsContent . view msg1004_observations
  sender            = toSender . view (msg1004_header . gpsObservationHeader_station)

instance FromObservations Msg1010 where
  gpsTimeNano m     = toGpsTimeNano (m ^. msg1010_header . glonassObservationHeader_station) $ rolloverEpochGpsTime (m ^. msg1010_header . glonassObservationHeader_epoch)
  packedObsContents = toPackedObsContent . view msg1010_observations
  sender            = toSender . view (msg1010_header . glonassObservationHeader_station)

instance FromObservations Msg1012 where
  gpsTimeNano m     = toGpsTimeNano (m ^. msg1012_header . glonassObservationHeader_station) $ rolloverEpochGpsTime (m ^. msg1012_header . glonassObservationHeader_epoch)
  packedObsContents = toPackedObsContent . view msg1012_observations
  sender            = toSender . view (msg1012_header . glonassObservationHeader_station)

-- | Convert RTCMv3 observations to SBP observations in chunks.
--
toMsgObs :: (MonadStore e m, FromObservations a) => a -> m [MsgObs]
toMsgObs m = do
  let obs = chunksOf maxObs $ packedObsContents m
  iforM obs $ \i obs' -> do
    t <- gpsTimeNano m
    let n = length obs `shiftL` 4 .|. i
    return $ MsgObs (ObservationHeader t (fromIntegral n)) obs'
  where
    maxObs  = (maxSize - hdrSize) `div` obsSize
    maxSize = 255
    hdrSize = 11
    obsSize = 17

-- | Convert RTCMv3 observation message to SBP observations message(s).
--
toSBPMsgObs :: (MonadStore e m, FromObservations a) => a -> m [SBPMsg]
toSBPMsgObs m = do
  ms <- toMsgObs m
  return $ for ms $ \m' ->
    SBPMsgObs m' $ toSBP m' $ sender m
