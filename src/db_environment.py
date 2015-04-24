#!/usr/bin/env python

from sqlalchemy import create_engine, event, orm
from sqlalchemy.orm import sessionmaker
from sqlalchemy.orm.session import Session as SessionBase, object_session
from sqlalchemy.event import listen
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy import inspect

class SignallingSession(SessionBase):
  def __init__(self, **options):
    self._model_changes = {}
    SessionBase.__init__(self, **options)

class _SessionSignalEvents(object):

  def register(self):
    listen(SessionBase, 'before_commit', self.session_signal_before_commit)
    listen(SessionBase, 'after_commit', self.session_signal_after_commit)
    listen(SessionBase, 'after_rollback', self.session_signal_after_rollback)

  @staticmethod
  def session_signal_before_commit(session):

    if not isinstance(session, SignallingSession):
      return

    d = session._model_changes
    if d:
      #print 'start before operations:',d
      for obj, change in d.values():
        if change == 'delete' and hasattr(obj, '__before_commit_delete__'):
         obj.__before_commit_delete__()
        elif change == 'insert' and hasattr(obj, '__before_commit_insert__'):
         obj.__before_commit_insert__()
        elif change == 'update' and hasattr(obj, '__before_commit_update__'):
         obj.__before_commit_update__()

      d.clear()
      #print 'end before operations:',d

  @staticmethod
  def session_signal_after_commit(session):
    if not isinstance(session, SignallingSession):
      return
    d = session._model_changes
    if d:
      #print 'start after operations:',d
      for obj, change in d.values():
        if change == 'delete' and hasattr(obj, '__after_commit_delete__'):
         obj.__after_commit_delete__()
        elif change == 'insert' and hasattr(obj, '__after_commit_insert__'):
         obj.__after_commit_insert__()
        elif change == 'update' and hasattr(obj, '__after_commit_update__'):
         obj.__after_commit_update__()

      d.clear()
      #print 'end after operations:',d

  @staticmethod
  def session_signal_after_rollback(session):
    if not isinstance(session, SignallingSession):
      return

    d = session._model_changes
    if d:
      d.clear()

class _MapperSignalEvents(object):

  def __init__(self, mapper):
    self.mapper = mapper

  def register(self):
    listen(self.mapper, 'before_delete', self.mapper_signal_before_insert)
    listen(self.mapper, 'before_insert', self.mapper_signal_before_insert)
    listen(self.mapper, 'before_update', self.mapper_signal_before_insert)

    listen(self.mapper, 'after_delete', self.mapper_signal_after_delete)
    listen(self.mapper, 'after_insert', self.mapper_signal_after_insert)
    listen(self.mapper, 'after_update', self.mapper_signal_after_update)

  def mapper_signal_before_insert(self, mapper, connection, target):
    self._record(mapper, target, 'delete')

  def mapper_signal_before_insert(self, mapper, connection, target):
    self._record(mapper, target, 'insert')

  def mapper_signal_before_insert(self, mapper, connection, target):
    self._record(mapper, target, 'update')

  def mapper_signal_after_delete(self, mapper, connection, target):
    self._record(mapper, target, 'delete')

  def mapper_signal_after_insert(self, mapper, connection, target):
    self._record(mapper, target, 'insert')

  def mapper_signal_after_update(self, mapper, connection, target):
    self._record(mapper, target, 'update')

  @staticmethod
  def _record(mapper, target, operation):
    s = object_session(target)
    if isinstance(s, SignallingSession):
      pk = tuple(mapper.primary_key_from_instance(target))
      s._model_changes[pk] = (target, operation)
    else:
      print 'everything so fucking wrong'

###
### Connection
###

Base = declarative_base()
engine = None
session = None

def initializeConnection(user, password, host, database, echo = False):

    global engine, session
    _SessionSignalEvents().register()
    _MapperSignalEvents(orm.mapper).register()

    engine = create_engine('postgresql://' + user + ':' + password + '@' + host + '/' + database, echo = echo)
    # use SignallingSession as session class
    session = sessionmaker(bind=engine, class_=SignallingSession)()
    print user, password, host, database

def db():
    global session
    return session

def engine():
    global engine
    return engine
